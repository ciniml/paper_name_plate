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

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct PlateContent {
    pub name: String,
    pub title: String,
    pub org: String,
    pub note: String,
    pub url: String,
}

impl PlateContent {
    pub fn demo() -> Self {
        Self {
            name: String::from("Kenta IDA"),
            title: String::from("Embedded Engineer"),
            org: String::from("bare-metal Rust / esp-hal"),
            note: String::from("Tap your phone to get the link"),
            url: String::from("github.com/esp-rs/esp-hal"),
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
            let mut hdr = 0x11 | 0x40; // SR, well-known, ME (last record)
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

/// Render the full name-plate screen.
pub fn draw(fb: &mut FrameBuffer, c: &PlateContent) {
    fb.clear(Gray2::WHITE).ok();

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
