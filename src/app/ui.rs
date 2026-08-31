//! Screen drawing for the demo application.

use alloc::string::String;
use core::fmt::Write as _;

use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_9X15};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Gray2;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, PrimitiveStyle, Rectangle, Triangle};
use embedded_graphics::text::Text;

use crate::ssd1677::{FrameBuffer, HEIGHT, WIDTH};
use crate::st25r3916::NfcaTag;

/// Tag panel area in logical (portrait) coordinates.
pub const PANEL_X: u32 = 12;
pub const PANEL_Y: u32 = 200;
pub const PANEL_W: u32 = WIDTH - 24;
pub const PANEL_H: u32 = 380;

pub fn hex(bytes: &[u8]) -> String {
    let mut s = String::with_capacity(bytes.len() * 3);
    for (i, b) in bytes.iter().enumerate() {
        if i > 0 {
            s.push(':');
        }
        let _ = write!(s, "{b:02X}");
    }
    s
}

/// Static part of the screen: title, gray ramp, status.
pub fn draw_base_screen(fb: &mut FrameBuffer, nfc_ok: bool) {
    fb.clear(Gray2::WHITE).ok();

    let bar_w = WIDTH / 4;
    for (i, level) in [0u8, 1, 2, 3].iter().enumerate() {
        let _ = Rectangle::new(Point::new((i as u32 * bar_w) as i32, 0), Size::new(bar_w, 40))
            .into_styled(PrimitiveStyle::with_fill(Gray2::new(*level)))
            .draw(fb);
    }
    let _ = Rectangle::new(Point::new(0, 0), Size::new(WIDTH, HEIGHT))
        .into_styled(PrimitiveStyle::with_stroke(Gray2::BLACK, 4))
        .draw(fb);

    let title = MonoTextStyle::new(&FONT_10X20, Gray2::BLACK);
    let _ = Text::new("M5Stack PaperMono", Point::new(120, 90), title).draw(fb);
    let _ = Text::new("bare-metal Rust / esp-hal", Point::new(90, 120), title).draw(fb);

    let small = MonoTextStyle::new(&FONT_9X15, Gray2::new(1));
    let _ = Text::new(
        if nfc_ok { "NFC reader ready - tap a card" } else { "NFC reader not available" },
        Point::new(20, 170),
        small,
    )
    .draw(fb);

    // Decorative: arrow + two gray circles at the bottom.
    let _ = Triangle::new(Point::new(240, 620), Point::new(200, 680), Point::new(280, 680))
        .into_styled(PrimitiveStyle::with_fill(Gray2::new(1)))
        .draw(fb);
    let _ = Circle::new(Point::new(60, 690), 90)
        .into_styled(PrimitiveStyle::with_fill(Gray2::new(1)))
        .draw(fb);
    let _ = Circle::new(Point::new(330, 690), 90)
        .into_styled(PrimitiveStyle::with_fill(Gray2::new(2)))
        .draw(fb);
}

/// Middle panel showing the last tag.
pub fn draw_tag_panel(fb: &mut FrameBuffer, tag: &NfcaTag, count: u32, t2: Option<&[u8; 16]>) {
    let area = Rectangle::new(Point::new(PANEL_X as i32, PANEL_Y as i32), Size::new(PANEL_W, PANEL_H));
    let _ = area.into_styled(PrimitiveStyle::with_fill(Gray2::WHITE)).draw(fb);
    let _ = area.into_styled(PrimitiveStyle::with_stroke(Gray2::new(1), 2)).draw(fb);

    let big = MonoTextStyle::new(&FONT_10X20, Gray2::BLACK);
    let small = MonoTextStyle::new(&FONT_9X15, Gray2::BLACK);

    let mut line = String::new();
    let _ = write!(line, "Tag #{count}");
    let _ = Text::new(&line, Point::new(24, 236), big).draw(fb);

    line.clear();
    let _ = write!(line, "UID: {}", hex(tag.uid()));
    let _ = Text::new(&line, Point::new(24, 276), small).draw(fb);

    line.clear();
    let _ = write!(line, "ATQA: {:04X}  SAK: {:02X}", tag.atqa, tag.sak);
    let _ = Text::new(&line, Point::new(24, 300), small).draw(fb);

    line.clear();
    let kind = if tag.is_iso14443_4() {
        "ISO14443-4 (DESFire/phone)"
    } else if tag.sak == 0x00 {
        "Type 2 (NTAG/Ultralight)"
    } else if tag.sak & 0x18 != 0 {
        "MIFARE Classic"
    } else {
        "NFC-A"
    };
    let _ = write!(line, "Type: {kind}");
    let _ = Text::new(&line, Point::new(24, 324), small).draw(fb);

    if let Some(d) = t2 {
        let _ = Text::new("Blocks 0-3:", Point::new(24, 364), small).draw(fb);
        for (i, chunk) in d.chunks(4).enumerate() {
            line.clear();
            let _ = write!(line, "{i}: {}", hex(chunk));
            let _ = Text::new(&line, Point::new(40, 388 + i as i32 * 22), small).draw(fb);
        }
    }
}
