//! SSD1677 e-paper controller driver for the PaperMono 3.97" 800x480 panel.
//!
//! Transport: 4-wire SPI (SCK/MOSI + D/C + CS), BUSY input (high = busy).
//! The hardware reset line and the panel power rail are *not* owned by this
//! driver (on PaperMono they hang off the M5IOE1 expander); the caller resets
//! the panel and then calls [`Ssd1677::init`].
//!
//! Refresh paths implemented:
//! * [`Ssd1677::refresh_mono_full`]   — built-in OTP mode-1 waveform, 1-bit image
//! * [`Ssd1677::refresh_mono_partial`] — built-in OTP partial waveform (fast, may ghost)
//! * [`Ssd1677::refresh_gray4`]       — custom LUT (`lut::LUT_QUALITY` / `LUT_TEXT`:
//!   mode 1, `LUT_FAST`: mode 2) with 4 gray levels
//!
//! The mode-1 / mode-2 RAM "face" bookkeeping follows M5GFX: every mode-2
//! activation swaps which RAM bank the controller treats as "current", which
//! exchanges the two middle grays for a subsequent mode-1 update.  We track
//! the parity and swap the planes accordingly.

pub mod framebuffer;
pub mod lut;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiBus;

pub use framebuffer::{FrameBuffer, Orientation, HEIGHT, PLANE_BYTES, ROW_BYTES, WIDTH};

/// Native panel geometry (controller RAM axes).
pub const RAM_X_PIXELS: u16 = 800; // source lines
pub const RAM_Y_LINES: u16 = 480; // gate lines

pub mod cmd {
    pub const DRIVER_OUTPUT: u8 = 0x01;
    pub const GATE_VOLTAGE: u8 = 0x03;
    pub const SOURCE_VOLTAGE: u8 = 0x04;
    pub const BOOSTER_SOFT_START: u8 = 0x0C;
    pub const DEEP_SLEEP: u8 = 0x10;
    pub const DATA_ENTRY: u8 = 0x11;
    pub const SW_RESET: u8 = 0x12;
    pub const TEMP_SENSOR: u8 = 0x18;
    pub const WRITE_TEMP: u8 = 0x1A;
    pub const MASTER_ACTIVATION: u8 = 0x20;
    pub const UPDATE_CTRL1: u8 = 0x21;
    pub const UPDATE_CTRL2: u8 = 0x22;
    pub const WRITE_RAM_BW: u8 = 0x24;
    pub const WRITE_RAM_RED: u8 = 0x26;
    pub const WRITE_VCOM: u8 = 0x2C;
    pub const WRITE_LUT: u8 = 0x32;
    pub const BORDER_WAVEFORM: u8 = 0x3C;
    pub const SET_RAM_X: u8 = 0x44;
    pub const SET_RAM_Y: u8 = 0x45;
    pub const AUTO_WRITE_RED: u8 = 0x46;
    pub const AUTO_WRITE_BW: u8 = 0x47;
    pub const SET_RAM_X_CNT: u8 = 0x4E;
    pub const SET_RAM_Y_CNT: u8 = 0x4F;
}

/// Bits of Display Update Control 2 (0x22).
pub mod ctrl2 {
    pub const ENABLE_CLOCK: u8 = 0x80;
    pub const ENABLE_ANALOG: u8 = 0x40;
    pub const LOAD_TEMP: u8 = 0x20;
    pub const LOAD_LUT: u8 = 0x10;
    pub const MODE2: u8 = 0x08;
    pub const DISPLAY: u8 = 0x04;
    pub const DISABLE_ANALOG: u8 = 0x02;
    pub const DISABLE_CLOCK: u8 = 0x01;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<S, P> {
    Spi(S),
    Pin(P),
    /// BUSY stayed high longer than the timeout.
    BusyTimeout,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GrayMode {
    /// Mode 1 with an erase prefix. Best quality, ~4.7 s.
    Quality,
    /// Mode 1, ~0.45 s.
    Text,
    /// Mode 2, ~0.34 s. Leaves the panel powered.
    Fast,
}

pub struct Ssd1677<SPI, DC, CS, BUSY> {
    spi: SPI,
    dc: DC,
    cs: CS,
    busy: BUSY,
    screen_on: bool,
    /// `true` when the controller's RAM face is swapped (odd number of mode-2
    /// activations since the last reset).
    face_odd: bool,
    face_known: bool,
    /// Optional per-plane scratch for inverted transfers (row sized).
    row: [u8; ROW_BYTES],
}

type Res<T, SPI, DC> = Result<T, Error<<SPI as embedded_hal::spi::ErrorType>::Error, <DC as embedded_hal::digital::ErrorType>::Error>>;

impl<SPI, DC, CS, BUSY> Ssd1677<SPI, DC, CS, BUSY>
where
    SPI: SpiBus<u8>,
    DC: OutputPin,
    CS: OutputPin<Error = DC::Error>,
    BUSY: InputPin<Error = DC::Error>,
{
    pub fn new(spi: SPI, dc: DC, cs: CS, busy: BUSY) -> Self {
        Self { spi, dc, cs, busy, screen_on: false, face_odd: false, face_known: false, row: [0; ROW_BYTES] }
    }

    // ------------------------------------------------------------------
    // low level
    // ------------------------------------------------------------------

    fn select(&mut self) -> Res<(), SPI, DC> {
        self.cs.set_low().map_err(Error::Pin)
    }

    fn deselect(&mut self) -> Res<(), SPI, DC> {
        self.cs.set_high().map_err(Error::Pin)
    }

    /// Send a command byte (CS handled here).
    pub fn command(&mut self, c: u8) -> Res<(), SPI, DC> {
        self.select()?;
        self.dc.set_low().map_err(Error::Pin)?;
        let r = self.spi.write(&[c]).map_err(Error::Spi);
        self.spi.flush().map_err(Error::Spi)?;
        self.deselect()?;
        r
    }

    /// Send data bytes (CS handled here).
    pub fn data(&mut self, d: &[u8]) -> Res<(), SPI, DC> {
        if d.is_empty() {
            return Ok(());
        }
        self.select()?;
        self.dc.set_high().map_err(Error::Pin)?;
        let r = self.spi.write(d).map_err(Error::Spi);
        self.spi.flush().map_err(Error::Spi)?;
        self.deselect()?;
        r
    }

    pub fn command_data(&mut self, c: u8, d: &[u8]) -> Res<(), SPI, DC> {
        self.command(c)?;
        self.data(d)
    }

    pub fn is_busy(&mut self) -> Res<bool, SPI, DC> {
        self.busy.is_high().map_err(Error::Pin)
    }

    /// Wait for BUSY to drop, polling every millisecond.
    pub fn wait_busy(&mut self, delay: &mut impl DelayNs, timeout_ms: u32) -> Res<(), SPI, DC> {
        // BUSY is not guaranteed to rise the instant the command finishes.
        delay.delay_ms(2);
        let mut waited = 0u32;
        while self.is_busy()? {
            if waited >= timeout_ms {
                return Err(Error::BusyTimeout);
            }
            delay.delay_ms(1);
            waited += 1;
        }
        Ok(())
    }

    // ------------------------------------------------------------------
    // init / power
    // ------------------------------------------------------------------

    /// Software reset + panel configuration. Call after a hardware reset
    /// (RST low >= 10 ms, then high >= 10 ms) with BUSY low.
    pub fn init(&mut self, delay: &mut impl DelayNs) -> Res<(), SPI, DC> {
        self.wait_busy(delay, 500)?;
        self.command(cmd::SW_RESET)?;
        delay.delay_ms(10);
        self.wait_busy(delay, 500)?;

        self.command_data(cmd::TEMP_SENSOR, &[0x80])?; // internal temperature sensor
        self.command_data(cmd::BOOSTER_SOFT_START, &[0xAE, 0xC7, 0xC3, 0xC0, 0x40])?;
        let gates = RAM_Y_LINES - 1;
        self.command_data(cmd::DRIVER_OUTPUT, &[(gates & 0xFF) as u8, (gates >> 8) as u8, 0x02])?;
        self.command_data(cmd::BORDER_WAVEFORM, &[0x01])?;
        self.command_data(cmd::UPDATE_CTRL1, &[0x00])?;

        // Full window; clear both RAM banks to white without a visible update.
        self.set_full_window()?;
        self.command_data(cmd::AUTO_WRITE_RED, &[0xF7])?;
        self.wait_busy(delay, 500)?;
        self.command_data(cmd::AUTO_WRITE_BW, &[0xF7])?;
        self.wait_busy(delay, 500)?;

        self.screen_on = false;
        self.face_odd = false;
        self.face_known = true;
        Ok(())
    }

    /// Switch the analog supply/clock on (0xC0) or off (0x03) without
    /// updating the image.
    pub fn set_power(&mut self, delay: &mut impl DelayNs, on: bool) -> Res<(), SPI, DC> {
        if on == self.screen_on {
            return Ok(());
        }
        self.command_data(cmd::UPDATE_CTRL2, &[if on { 0xC0 } else { 0x03 }])?;
        self.command(cmd::MASTER_ACTIVATION)?;
        self.wait_busy(delay, 2000)?;
        self.screen_on = on;
        Ok(())
    }

    /// Power down and enter deep sleep mode 1 (RAM retained). Leaving deep
    /// sleep requires a hardware reset followed by [`Self::init`].
    pub fn deep_sleep(&mut self, delay: &mut impl DelayNs) -> Res<(), SPI, DC> {
        self.set_power(delay, false)?;
        self.command_data(cmd::DEEP_SLEEP, &[0x01])?;
        self.face_known = false;
        Ok(())
    }

    // ------------------------------------------------------------------
    // RAM addressing
    // ------------------------------------------------------------------

    /// Program a RAM window in native coordinates (`x` in source pixels,
    /// multiple of 8; `y` in gate lines) with X++/Y++ data entry.
    pub fn set_window(&mut self, x: u16, y: u16, w: u16, h: u16) -> Res<(), SPI, DC> {
        let xe = x + w - 1;
        let ye = y + h - 1;
        self.command_data(cmd::DATA_ENTRY, &[0x03])?;
        self.command_data(cmd::SET_RAM_X, &[(x & 0xFF) as u8, (x >> 8) as u8, (xe & 0xFF) as u8, (xe >> 8) as u8])?;
        self.command_data(cmd::SET_RAM_Y, &[(y & 0xFF) as u8, (y >> 8) as u8, (ye & 0xFF) as u8, (ye >> 8) as u8])?;
        self.command_data(cmd::SET_RAM_X_CNT, &[(x & 0xFF) as u8, (x >> 8) as u8])?;
        self.command_data(cmd::SET_RAM_Y_CNT, &[(y & 0xFF) as u8, (y >> 8) as u8])
    }

    pub fn set_full_window(&mut self) -> Res<(), SPI, DC> {
        self.set_window(0, 0, RAM_X_PIXELS, RAM_Y_LINES)
    }

    /// Write a full plane (framebuffer RAM order) to `ram_cmd` (0x24 / 0x26),
    /// optionally bit-inverted.
    pub fn write_plane(&mut self, ram_cmd: u8, plane: &[u8; PLANE_BYTES], invert: bool) -> Res<(), SPI, DC> {
        self.set_full_window()?;
        self.command(ram_cmd)?;
        if !invert {
            return self.data(plane);
        }
        self.select()?;
        self.dc.set_high().map_err(Error::Pin)?;
        for row in plane.chunks_exact(ROW_BYTES) {
            for (d, s) in self.row.iter_mut().zip(row) {
                *d = !*s;
            }
            self.spi.write(&self.row).map_err(Error::Spi)?;
        }
        self.spi.flush().map_err(Error::Spi)?;
        self.deselect()
    }

    /// Write a rectangular part of a plane. `x0` is given in native source
    /// pixels and rounded outwards to byte boundaries; `y0..y0+h` are gate rows
    /// (= framebuffer rows).
    #[allow(clippy::too_many_arguments)]
    pub fn write_plane_rect(
        &mut self,
        ram_cmd: u8,
        plane: &[u8; PLANE_BYTES],
        x0: u16,
        y0: u16,
        w: u16,
        h: u16,
        invert: bool,
    ) -> Res<(), SPI, DC> {
        let xs = x0 & !7;
        let xe = ((x0 + w - 1) | 7).min(RAM_X_PIXELS - 1);
        let bytes = ((xe - xs + 1) / 8) as usize;
        let first = (xs / 8) as usize;
        self.set_window(xs, y0, xe - xs + 1, h)?;
        self.command(ram_cmd)?;
        self.select()?;
        self.dc.set_high().map_err(Error::Pin)?;
        for y in y0..y0 + h {
            let row = &plane[y as usize * ROW_BYTES + first..][..bytes];
            if invert {
                for (d, s) in self.row[..bytes].iter_mut().zip(row) {
                    *d = !*s;
                }
                self.spi.write(&self.row[..bytes]).map_err(Error::Spi)?;
            } else {
                self.spi.write(row).map_err(Error::Spi)?;
            }
        }
        self.spi.flush().map_err(Error::Spi)?;
        self.deselect()
    }

    // ------------------------------------------------------------------
    // activation
    // ------------------------------------------------------------------

    fn activate(&mut self, delay: &mut impl DelayNs, ctrl1: u8, mut c2: u8, timeout_ms: u32) -> Res<(), SPI, DC> {
        self.command_data(cmd::UPDATE_CTRL1, &[ctrl1])?;
        if !self.screen_on {
            c2 |= ctrl2::ENABLE_CLOCK | ctrl2::ENABLE_ANALOG;
        }
        self.command_data(cmd::UPDATE_CTRL2, &[c2])?;
        self.command(cmd::MASTER_ACTIVATION)?;
        self.wait_busy(delay, timeout_ms)?;
        let powers_down = c2 & (ctrl2::DISABLE_ANALOG | ctrl2::DISABLE_CLOCK) != 0;
        self.screen_on = !powers_down;
        if c2 & ctrl2::MODE2 != 0 {
            if self.face_known {
                self.face_odd = !self.face_odd;
            }
        } else {
            self.face_odd = false;
            self.face_known = true;
        }
        Ok(())
    }

    fn ensure_known_face(&mut self, delay: &mut impl DelayNs) -> Res<(), SPI, DC> {
        if self.face_known {
            return Ok(());
        }
        self.init(delay)
    }

    fn send_lut(&mut self, lut: &[u8; lut::LUT_LEN]) -> Res<(), SPI, DC> {
        self.command_data(cmd::WRITE_LUT, &lut[..105])?;
        self.command_data(cmd::GATE_VOLTAGE, &[lut[105]])?;
        self.command_data(cmd::SOURCE_VOLTAGE, &[lut[106], lut[107], lut[108]])?;
        self.command_data(cmd::WRITE_VCOM, &[lut[109]])
    }

    // ------------------------------------------------------------------
    // refresh paths
    // ------------------------------------------------------------------

    /// Full-screen monochrome refresh with the controller's OTP mode-1
    /// waveform. `mono` is 1 = white. Both RAM banks receive the image so it
    /// also becomes the baseline for [`Self::refresh_mono_partial`].
    pub fn refresh_mono_full(&mut self, delay: &mut impl DelayNs, mono: &[u8; PLANE_BYTES]) -> Res<(), SPI, DC>
    where
        SPI: SpiBus<u8>,
    {
        self.ensure_known_face(delay)?;
        self.write_plane(cmd::WRITE_RAM_RED, mono, false)?;
        self.write_plane(cmd::WRITE_RAM_BW, mono, false)?;
        // load temp + load LUT + display, mode 1, then keep the panel powered.
        self.activate(delay, 0x00, ctrl2::LOAD_TEMP | ctrl2::LOAD_LUT | ctrl2::DISPLAY, 10_000)
    }

    /// Partial monochrome update with the OTP "partial" sequence (0xFF).
    /// Requires a monochrome baseline in RAM (from `refresh_mono_full` or a
    /// previous partial). Only rows `y0..y0+h` / columns `x0..x0+w` (native
    /// coordinates) are transferred; the whole panel is activated.
    pub fn refresh_mono_partial(
        &mut self,
        delay: &mut impl DelayNs,
        mono: &[u8; PLANE_BYTES],
        x0: u16,
        y0: u16,
        w: u16,
        h: u16,
    ) -> Res<(), SPI, DC> {
        self.ensure_known_face(delay)?;
        self.command_data(cmd::BORDER_WAVEFORM, &[0x80])?; // float border during partial
        self.write_plane_rect(cmd::WRITE_RAM_BW, mono, x0, y0, w, h, false)?;
        // 0xFF = OTP partial sequence; clock/analog enable bits are added by activate() when needed.
        self.activate(delay, 0x00, 0x3F, 5_000)?;
        // Keep RED (previous image) in sync so the next partial diff is right.
        self.write_plane_rect(cmd::WRITE_RAM_RED, mono, x0, y0, w, h, false)?;
        self.command_data(cmd::BORDER_WAVEFORM, &[0x01])
    }

    /// Full-screen 4-gray refresh using one of the custom LUTs.
    ///
    /// Planes are the framebuffer's `lsb` / `msb` (level 0 = black .. 3 =
    /// white); they are sent inverted so that the RAM group index equals
    /// `3 - level` (0 = white .. 3 = black) as the LUTs expect.
    pub fn refresh_gray4(
        &mut self,
        delay: &mut impl DelayNs,
        lsb: &[u8; PLANE_BYTES],
        msb: &[u8; PLANE_BYTES],
        mode: GrayMode,
    ) -> Res<(), SPI, DC> {
        self.ensure_known_face(delay)?;
        match mode {
            GrayMode::Quality | GrayMode::Text => {
                // Mode 1: the ping-pong face exchanges the two middle codes;
                // swap plane destinations on the odd face.
                let (bw, red) = if self.face_odd { (msb, lsb) } else { (lsb, msb) };
                self.write_plane(cmd::WRITE_RAM_BW, bw, true)?;
                self.write_plane(cmd::WRITE_RAM_RED, red, true)?;
                let table = if matches!(mode, GrayMode::Quality) { &lut::LUT_QUALITY } else { &lut::LUT_TEXT };
                self.send_lut(table)?;
                // display + disable analog + disable clock (panel ends powered down)
                self.activate(delay, 0x00, ctrl2::DISPLAY | ctrl2::DISABLE_ANALOG | ctrl2::DISABLE_CLOCK, 15_000)
            }
            GrayMode::Fast => {
                self.write_plane(cmd::WRITE_RAM_BW, lsb, true)?;
                self.write_plane(cmd::WRITE_RAM_RED, msb, true)?;
                self.send_lut(&lut::LUT_FAST)?;
                self.activate(delay, 0x00, ctrl2::MODE2 | ctrl2::DISPLAY, 5_000)
            }
        }
    }

    /// Convenience: full 4-gray refresh straight from a framebuffer.
    pub fn display_gray4(&mut self, delay: &mut impl DelayNs, fb: &FrameBuffer, mode: GrayMode) -> Res<(), SPI, DC> {
        self.refresh_gray4(delay, &fb.lsb, &fb.msb, mode)
    }

    /// Convenience: full monochrome refresh from a framebuffer (levels >= 2 are white).
    pub fn display_mono(&mut self, delay: &mut impl DelayNs, fb: &FrameBuffer) -> Res<(), SPI, DC> {
        self.refresh_mono_full(delay, fb.mono_plane())
    }
}
