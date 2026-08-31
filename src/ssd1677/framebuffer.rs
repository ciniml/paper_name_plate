//! Two-plane 4-gray framebuffer for the 480x800 (portrait) PaperMono display.
//!
//! Memory layout is the controller's RAM order so a whole plane can be sent
//! with one `0x24` / `0x26` write and data-entry mode `0x03` (X++, Y++):
//!
//! * RAM Y (gate, 0..480)   == logical **x** (portrait column)  -> row index
//! * RAM X (source, 0..800) == logical **y** (portrait row)     -> bit index,
//!   8 pixels per byte, MSB first.
//!
//! `byte = x * ROW_BYTES + y / 8`, `bit = 0x80 >> (y % 8)`.
//!
//! Each pixel stores a gray level `v` in 0..=3 (0 = black, 3 = white) split
//! into `lsb = v & 1` and `msb = v >> 1` planes.  The `msb` plane alone is a
//! monochrome image (white if v >= 2) and is what the monochrome refresh paths
//! send to the controller.

use embedded_graphics::pixelcolor::Gray2;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;

/// Portrait width (logical x).
pub const WIDTH: u32 = 480;
/// Portrait height (logical y).
pub const HEIGHT: u32 = 800;
/// Bytes per RAM row (800 source pixels / 8).
pub const ROW_BYTES: usize = (HEIGHT as usize) / 8;
/// Bytes per bit-plane.
pub const PLANE_BYTES: usize = ROW_BYTES * WIDTH as usize;

/// Axis flips applied when mapping logical coordinates to controller RAM.
/// Adjust once the panel orientation has been verified on hardware.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Orientation {
    pub flip_x: bool,
    pub flip_y: bool,
}

pub struct FrameBuffer {
    pub lsb: [u8; PLANE_BYTES],
    pub msb: [u8; PLANE_BYTES],
    pub orientation: Orientation,
}

impl FrameBuffer {
    /// All white.
    pub const fn new() -> Self {
        Self {
            lsb: [0xFF; PLANE_BYTES],
            msb: [0xFF; PLANE_BYTES],
            orientation: Orientation { flip_x: false, flip_y: false },
        }
    }

    pub fn clear_white(&mut self) {
        self.lsb.fill(0xFF);
        self.msb.fill(0xFF);
    }

    pub fn clear_black(&mut self) {
        self.lsb.fill(0x00);
        self.msb.fill(0x00);
    }

    #[inline]
    fn locate(&self, x: u32, y: u32) -> (usize, u8) {
        let x = if self.orientation.flip_x { WIDTH - 1 - x } else { x };
        let y = if self.orientation.flip_y { HEIGHT - 1 - y } else { y };
        ((x as usize) * ROW_BYTES + (y as usize >> 3), 0x80 >> (y & 7))
    }

    /// Set a pixel to gray level 0..=3 (0 = black). Out-of-range is ignored.
    #[inline]
    pub fn set_pixel(&mut self, x: i32, y: i32, level: u8) {
        if x < 0 || y < 0 || x as u32 >= WIDTH || y as u32 >= HEIGHT {
            return;
        }
        let (idx, bit) = self.locate(x as u32, y as u32);
        if level & 1 != 0 { self.lsb[idx] |= bit } else { self.lsb[idx] &= !bit }
        if level & 2 != 0 { self.msb[idx] |= bit } else { self.msb[idx] &= !bit }
    }

    #[inline]
    pub fn get_pixel(&self, x: u32, y: u32) -> u8 {
        let (idx, bit) = self.locate(x, y);
        (self.lsb[idx] & bit != 0) as u8 | (((self.msb[idx] & bit != 0) as u8) << 1)
    }

    /// The monochrome view of the image (white if level >= 2).
    pub fn mono_plane(&self) -> &[u8; PLANE_BYTES] {
        &self.msb
    }

    /// Copy the full image from `src` (used to track what is on glass).
    pub fn copy_from(&mut self, src: &FrameBuffer) {
        self.lsb.copy_from_slice(&src.lsb);
        self.msb.copy_from_slice(&src.msb);
    }

    /// Fold a monochrome update into this buffer: for the given *native*
    /// region (rows = logical x, byte-aligned bit columns = logical y), set
    /// both planes to `mono` so the buffer matches the glass after a
    /// `refresh_fastest`.
    pub fn apply_mono_region(&mut self, mono: &[u8; PLANE_BYTES], x0: u16, w: u16, y0: u16, h: u16) {
        let first = (x0 & !7) as usize / 8;
        let last = (((x0 + w - 1) | 7).min(HEIGHT as u16 - 1)) as usize / 8;
        let ys = y0 as usize;
        let ye = (y0 + h - 1).min(WIDTH as u16 - 1) as usize;
        for y in ys..=ye {
            let o = y * ROW_BYTES;
            self.lsb[o + first..=o + last].copy_from_slice(&mono[o + first..=o + last]);
            self.msb[o + first..=o + last].copy_from_slice(&mono[o + first..=o + last]);
        }
    }

    /// Map a logical rectangle (portrait coords) to the native region used by
    /// `refresh_fastest` / `apply_mono_region`: returns (x0_bits, w_bits,
    /// y0_rows, h_rows). Assumes no orientation flips.
    pub fn native_region(x: u32, y: u32, w: u32, h: u32) -> (u16, u16, u16, u16) {
        let x = x.min(WIDTH - 1);
        let y = y.min(HEIGHT - 1);
        let w = w.min(WIDTH - x);
        let h = h.min(HEIGHT - y);
        (y as u16, h as u16, x as u16, w as u16)
    }
}

impl Default for FrameBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl OriginDimensions for FrameBuffer {
    fn size(&self) -> Size {
        Size::new(WIDTH, HEIGHT)
    }
}

impl DrawTarget for FrameBuffer {
    type Color = Gray2;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(p, c) in pixels {
            self.set_pixel(p.x, p.y, c.luma());
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        let area = area.intersection(&self.bounding_box());
        if area.is_zero_sized() {
            return Ok(());
        }
        let level = color.luma();
        // Rows of the framebuffer are logical x; a full-height column fill is
        // contiguous, everything else goes through set_pixel.
        for x in area.columns() {
            for y in area.rows() {
                self.set_pixel(x, y, level);
            }
        }
        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let v = color.luma();
        self.lsb.fill(if v & 1 != 0 { 0xFF } else { 0x00 });
        self.msb.fill(if v & 2 != 0 { 0xFF } else { 0x00 });
        Ok(())
    }
}
