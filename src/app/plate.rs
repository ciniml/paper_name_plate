//! The name plate itself: content model, NDEF mapping and rendering.
//!
//! Content convention (writable from a phone with any NFC tag writer):
//! * a Text record ("T") holds up to four lines: name / title / organisation
//!   / extra note (separated by `\n`)
//! * a URI record ("U") holds the link that is also served to readers
//!
//! The same NDEF message is what the emulated tag serves, so whatever a
//! phone writes is what the next phone reads.

use alloc::string::String;
use alloc::vec::Vec;

use embedded_graphics::pixelcolor::Gray2;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use u8g2_fonts::types::{FontColor, HorizontalAlignment, VerticalPosition};
use u8g2_fonts::{fonts, FontRenderer};

use crate::ssd1677::{FrameBuffer, HEIGHT, WIDTH};

/// Draw-target adapter that renders every pixel as a 2x2 block, giving a
/// poor-man's 32 px Japanese font from the 16 px unifont.
struct Scale2x<'a> {
    fb: &'a mut FrameBuffer,
}

impl OriginDimensions for Scale2x<'_> {
    fn size(&self) -> Size {
        Size::new(WIDTH / 2, HEIGHT / 2)
    }
}

impl DrawTarget for Scale2x<'_> {
    type Color = Gray2;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(p, c) in pixels {
            let (x, y) = (p.x * 2, p.y * 2);
            self.fb.set_pixel(x, y, c.luma());
            self.fb.set_pixel(x + 1, y, c.luma());
            self.fb.set_pixel(x, y + 1, c.luma());
            self.fb.set_pixel(x + 1, y + 1, c.luma());
        }
        Ok(())
    }
}

fn is_plain_ascii(s: &str) -> bool {
    s.bytes().all(|b| (0x20..0x7F).contains(&b))
}

/// MIME type of the plate's image record.
pub const IMAGE_MIME: &[u8] = b"image/x-plate";

/// A 1-bpp image: rows padded to whole bytes, MSB first, 1 = black.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct MonoImage {
    pub width: u16,
    pub height: u16,
    pub bits: Vec<u8>,
}

impl MonoImage {
    pub fn row_bytes(&self) -> usize {
        (self.width as usize).div_ceil(8)
    }

    /// Parse `[w u16 LE][h u16 LE][rows]`; size-checked.
    pub fn decode(payload: &[u8]) -> Option<Self> {
        if payload.len() < 4 {
            return None;
        }
        let width = u16::from_le_bytes([payload[0], payload[1]]);
        let height = u16::from_le_bytes([payload[2], payload[3]]);
        if width == 0 || height == 0 || width > 480 || height > 800 {
            return None;
        }
        let need = (width as usize).div_ceil(8) * height as usize;
        let bits = payload.get(4..4 + need)?.to_vec();
        Some(Self { width, height, bits })
    }

    pub fn encode(&self) -> Vec<u8> {
        let mut v = Vec::with_capacity(4 + self.bits.len());
        v.extend_from_slice(&self.width.to_le_bytes());
        v.extend_from_slice(&self.height.to_le_bytes());
        v.extend_from_slice(&self.bits);
        v
    }
}

/// Minimal base64 (standard alphabet, '=' padding, whitespace ignored).
fn b64_decode(s: &[u8]) -> Option<Vec<u8>> {
    fn val(c: u8) -> Option<u8> {
        match c {
            b'A'..=b'Z' => Some(c - b'A'),
            b'a'..=b'z' => Some(c - b'a' + 26),
            b'0'..=b'9' => Some(c - b'0' + 52),
            b'+' => Some(62),
            b'/' => Some(63),
            _ => None,
        }
    }
    let mut out = Vec::new();
    let mut acc: u32 = 0;
    let mut n = 0;
    for &c in s {
        if c.is_ascii_whitespace() || c == b'=' {
            continue;
        }
        acc = (acc << 6) | val(c)? as u32;
        n += 1;
        if n == 4 {
            out.extend_from_slice(&[(acc >> 16) as u8, (acc >> 8) as u8, acc as u8]);
            acc = 0;
            n = 0;
        }
    }
    match n {
        0 => {}
        2 => out.push((acc >> 4) as u8),
        3 => {
            out.push((acc >> 10) as u8);
            out.push((acc >> 2) as u8);
        }
        _ => return None,
    }
    Some(out)
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct PlateContent {
    pub name: String,
    pub title: String,
    pub org: String,
    pub note: String,
    pub url: String,
    pub image: Option<MonoImage>,
}

impl PlateContent {
    pub fn demo() -> Self {
        Self {
            name: String::from("Kenta IDA"),
            title: String::from("Embedded Engineer"),
            org: String::from("bare-metal Rust / esp-hal"),
            note: String::from("Tap your phone to get the link"),
            url: String::from("github.com/esp-rs/esp-hal"),
            image: None,
        }
    }

    /// Build the NDEF message served by the emulated tag:
    /// URI record first (phones act on the first record), then the text.
    pub fn to_ndef(&self) -> Vec<u8> {
        let mut msg = Vec::new();
        let text = self.text_payload();
        let n_records = 1 + !self.url.is_empty() as usize;
        let mut first = true;
        if !self.url.is_empty() {
            let mut hdr = 0x11; // SR, TNF well-known
            if first {
                hdr |= 0x80;
            }
            if n_records == 1 {
                hdr |= 0x40;
            }
            msg.push(hdr);
            msg.push(0x01);
            msg.push((self.url.len() + 1) as u8);
            msg.push(b'U');
            msg.push(0x04); // https://
            msg.extend_from_slice(self.url.as_bytes());
            first = false;
        }
        {
            let last = self.image.is_none();
            let mut hdr = 0x11; // SR, well-known
            if last {
                hdr |= 0x40;
            }
            if first {
                hdr |= 0x80;
            }
            msg.push(hdr);
            msg.push(0x01);
            msg.push((text.len() + 3) as u8);
            msg.push(b'T');
            msg.push(0x02); // UTF-8, lang length 2
            msg.extend_from_slice(b"en");
            msg.extend_from_slice(&text);
        }
        if let Some(img) = &self.image {
            let payload = img.encode();
            if payload.len() > 700 {
                // Too big for the NTAG user area: display-only.
                if let Some(last) = msg.first_mut() {
                    let _ = last;
                }
                // Fix the ME flag of the text record (it was cleared above).
                fix_me_flag(&mut msg);
                return msg;
            }
            msg.push(0x12 | 0x40); // SR, MIME (TNF 2), ME
            msg.push(IMAGE_MIME.len() as u8);
            if payload.len() < 256 {
                msg.push(payload.len() as u8);
            } else {
                // Long record: clear SR, 4-byte length.
                let idx = msg.len() - 2;
                msg[idx] &= !0x10;
                msg.extend_from_slice(&(payload.len() as u32).to_be_bytes());
            }
            msg.extend_from_slice(IMAGE_MIME);
            msg.extend_from_slice(&payload);
        }
        msg
    }

    fn text_payload(&self) -> Vec<u8> {
        let mut t = Vec::new();
        for (i, part) in [&self.name, &self.title, &self.org, &self.note].iter().enumerate() {
            if i > 0 {
                t.push(b'\n');
            }
            t.extend_from_slice(part.as_bytes());
        }
        t
    }

    /// Update the content from an NDEF message (all records scanned).
    pub fn apply_ndef(&mut self, msg: &[u8]) {
        let mut i = 0usize;
        while let Some((tnf, rtype, payload, next)) = record_at(msg, i) {
            match (tnf, rtype) {
                (1, b"T") => {
                    let lang_len = (payload.first().copied().unwrap_or(0) & 0x3F) as usize;
                    if let Some(text) = payload.get(1 + lang_len..) {
                        // Some NFC writer apps make newlines hard to type:
                        // accept ';' as an alternative separator.
                        let text = String::from_utf8_lossy(text).replace(';', "\n");
                        let mut lines = text.lines();
                        self.name = lines.next().unwrap_or("").into();
                        self.title = lines.next().unwrap_or("").into();
                        self.org = lines.next().unwrap_or("").into();
                        self.note = lines.next().unwrap_or("").into();
                    }
                }
                (2, t) if t == IMAGE_MIME => {
                    // Raw binary, or "B64:<base64>" typed into an NFC writer.
                    let decoded;
                    let raw = if payload.starts_with(b"B64:") {
                        decoded = b64_decode(&payload[4..]);
                        decoded.as_deref()
                    } else {
                        Some(payload)
                    };
                    if let Some(img) = raw.and_then(MonoImage::decode) {
                        log::info!("plate: image {}x{} ({} B)", img.width, img.height, img.bits.len());
                        self.image = Some(img);
                    } else {
                        log::warn!("plate: bad image record ({} B)", payload.len());
                    }
                }
                (1, b"U") => {
                    if let Some((&prefix, rest)) = payload.split_first() {
                        let mut url = String::new();
                        url.push_str(crate::ndef::uri_prefix(prefix));
                        url.push_str(&String::from_utf8_lossy(rest));
                        // Store without scheme for display compactness.
                        self.url = url.trim_start_matches("https://").trim_start_matches("http://").into();
                    }
                }
                _ => {}
            }
            if next <= i || next >= msg.len() {
                break;
            }
            i = next;
        }
    }
}

/// Set the ME (message end) flag on the last record header of `msg`.
fn fix_me_flag(msg: &mut [u8]) {
    let mut i = 0usize;
    let mut last_hdr = None;
    while i < msg.len() {
        last_hdr = Some(i);
        let Some((_, _, _, next)) = record_at(msg, i) else { break };
        if next <= i || next >= msg.len() {
            break;
        }
        i = next;
    }
    if let Some(h) = last_hdr {
        msg[h] |= 0x40;
    }
}

/// Parse the record at byte offset `i`; returns (tnf, type, payload, next offset).
fn record_at(msg: &[u8], i: usize) -> Option<(u8, &[u8], &[u8], usize)> {
    let hdr = *msg.get(i)?;
    let tnf = hdr & 0x07;
    let sr = hdr & 0x10 != 0;
    let il = hdr & 0x08 != 0;
    let mut p = i + 1;
    let type_len = *msg.get(p)? as usize;
    p += 1;
    let payload_len = if sr {
        let l = *msg.get(p)? as usize;
        p += 1;
        l
    } else {
        let b = msg.get(p..p + 4)?;
        p += 4;
        u32::from_be_bytes([b[0], b[1], b[2], b[3]]) as usize
    };
    if il {
        let idl = *msg.get(p)? as usize;
        p += 1 + idl;
    }
    let rtype = msg.get(p..p + type_len)?;
    p += type_len;
    let payload = msg.get(p..p + payload_len)?;
    Some((tnf, rtype, payload, p + payload_len))
}

/// Draw `img` centered at (`cx`, `cy`), black pixels only.
fn draw_image(fb: &mut FrameBuffer, img: &MonoImage, cx: i32, cy: i32, scale: i32) {
    let x0 = cx - (img.width as i32 * scale) / 2;
    let y0 = cy - (img.height as i32 * scale) / 2;
    let rb = img.row_bytes();
    for y in 0..img.height as i32 {
        for x in 0..img.width as i32 {
            let bit = img.bits[y as usize * rb + (x as usize >> 3)] & (0x80 >> (x & 7));
            if bit != 0 {
                for dy in 0..scale {
                    for dx in 0..scale {
                        fb.set_pixel(x0 + x * scale + dx, y0 + y * scale + dy, 0);
                    }
                }
            }
        }
    }
}

/// Render the full name-plate screen.
pub fn draw(fb: &mut FrameBuffer, c: &PlateContent) {
    fb.clear(Gray2::WHITE).ok();

    // A large (near full-screen) image replaces the whole plate layout.
    if let Some(img) = &c.image
        && img.width >= 400
    {
        draw_image(fb, img, WIDTH as i32 / 2, HEIGHT as i32 / 2, 1);
        return;
    }

    let big = FontRenderer::new::<fonts::u8g2_font_logisoso42_tf>();
    let mid = FontRenderer::new::<fonts::u8g2_font_logisoso22_tf>();
    let small = FontRenderer::new::<fonts::u8g2_font_helvR14_tf>();
    // 16 px font covering kana + JIS level 1/2 kanji (rendered 2x for names).
    let jp = FontRenderer::new::<fonts::u8g2_font_b16_t_japanese2>();

    // Header band.
    let _ = Rectangle::new(Point::zero(), Size::new(WIDTH, 60))
        .into_styled(PrimitiveStyle::with_fill(Gray2::BLACK))
        .draw(fb);
    let _ = small.render_aligned(
        "PAPER NAME PLATE",
        Point::new(WIDTH as i32 / 2, 38),
        VerticalPosition::Baseline,
        HorizontalAlignment::Center,
        FontColor::Transparent(Gray2::WHITE),
        fb,
    );

    let cx = WIDTH as i32 / 2;
    if is_plain_ascii(&c.name) {
        let _ = big.render_aligned(
            c.name.as_str(),
            Point::new(cx, 260),
            VerticalPosition::Baseline,
            HorizontalAlignment::Center,
            FontColor::Transparent(Gray2::BLACK),
            fb,
        );
    } else {
        // Japanese name: 16 px font at 2x = 32 px, in scaled coordinates.
        let _ = jp.render_aligned(
            c.name.as_str(),
            Point::new(cx / 2, 130),
            VerticalPosition::Baseline,
            HorizontalAlignment::Center,
            FontColor::Transparent(Gray2::BLACK),
            &mut Scale2x { fb },
        );
    }
    for (text, y) in [(&c.title, 330), (&c.org, 375)] {
        if text.is_empty() {
            continue;
        }
        if is_plain_ascii(text) {
            let _ = mid.render_aligned(
                text.as_str(),
                Point::new(cx, y),
                VerticalPosition::Baseline,
                HorizontalAlignment::Center,
                FontColor::Transparent(Gray2::new(1)),
                fb,
            );
        } else {
            let _ = jp.render_aligned(
                text.as_str(),
                Point::new(cx, y),
                VerticalPosition::Baseline,
                HorizontalAlignment::Center,
                FontColor::Transparent(Gray2::new(1)),
                fb,
            );
        }
    }

    // Divider.
    let _ = Rectangle::new(Point::new(60, 430), Size::new(WIDTH - 120, 3))
        .into_styled(PrimitiveStyle::with_fill(Gray2::new(2)))
        .draw(fb);

    if !c.url.is_empty() {
        let _ = small.render_aligned(
            c.url.as_str(),
            Point::new(cx, 490),
            VerticalPosition::Baseline,
            HorizontalAlignment::Center,
            FontColor::Transparent(Gray2::BLACK),
            fb,
        );
    }
    if !c.note.is_empty() {
        let f = if is_plain_ascii(&c.note) { &small } else { &jp };
        let _ = f.render_aligned(
            c.note.as_str(),
            Point::new(cx, 540),
            VerticalPosition::Baseline,
            HorizontalAlignment::Center,
            FontColor::Transparent(Gray2::new(1)),
            fb,
        );
    }

    if let Some(img) = &c.image {
        // Centered, 2x-scaled when small, in the area below the note.
        let scale: i32 = if img.width <= 120 && img.height <= 100 { 2 } else { 1 };
        draw_image(fb, img, cx, 660, scale);
    }

    // Footer: NFC hint.
    let _ = small.render_aligned(
        "NFC: tap to receive / write to update",
        Point::new(cx, HEIGHT as i32 - 30),
        VerticalPosition::Baseline,
        HorizontalAlignment::Center,
        FontColor::Transparent(Gray2::new(1)),
        fb,
    );
}
