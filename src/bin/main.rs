#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::cell::RefCell;
use core::fmt::Write as _;

use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_9X15};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Gray2;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, PrimitiveStyle, Rectangle, Triangle};
use embedded_graphics::text::Text;
use embedded_hal_bus::i2c::RefCellDevice;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::main;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::spi::Mode as SpiMode;
use esp_hal::time::{Instant, Rate};
use log::{error, info, warn};
use static_cell::StaticCell;

use paper_name_plate::board;
use paper_name_plate::ioe1::Ioe1;
use paper_name_plate::pm1::Pm1;
use paper_name_plate::ssd1677::{FrameBuffer, GrayMode, Ssd1677, HEIGHT, WIDTH};
use paper_name_plate::st25r3916::{NfcaTag, St25r3916};

extern crate alloc;
use alloc::string::String;

esp_bootloader_esp_idf::esp_app_desc!();

static FRAMEBUFFER: StaticCell<FrameBuffer> = StaticCell::new();

#[allow(clippy::large_stack_frames, reason = "main owns the peripherals")]
#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let p = esp_hal::init(config);
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);

    let mut delay = Delay::new();
    info!("PaperMono bare-metal Rust: boot");

    // ---------------------------------------------------------------
    // Internal I2C bus (PM1, IOE1, touch, NFC, RTC, IMU) @ 100 kHz.
    // PM1/IOE1 default to 100 kHz and remember their SPD (400 kHz) bit across
    // ESP32 resets (the PMIC is always powered), so the bus is kept at 100 kHz
    // and `recover_i2c_speed` puts stray 400 kHz-mode devices back.
    // ---------------------------------------------------------------
    let i2c = I2c::new(p.I2C0, I2cConfig::default().with_frequency(Rate::from_khz(100)))
        .expect("i2c config")
        .with_sda(p.GPIO47)
        .with_scl(p.GPIO48);
    let i2c_bus = RefCell::new(i2c);

    // ---------------------------------------------------------------
    // PMIC
    // ---------------------------------------------------------------
    let mut pm1 = Pm1::new(RefCellDevice::new(&i2c_bus));
    if pm1.init(&mut delay).is_err() {
        warn!("PM1 init failed at 100 kHz; resetting PM1/IOE1 I2C speed");
        recover_i2c_speed(&i2c_bus, &mut delay);
    }
    match pm1.init(&mut delay) {
        Ok(()) => {
            let hw = pm1.hw_rev().unwrap_or(0);
            let sw = pm1.sw_rev().unwrap_or(0);
            let src = pm1.power_source().unwrap_or(0xFF);
            let vbat = pm1.battery_mv().unwrap_or(0);
            let vin = pm1.vin_mv().unwrap_or(0);
            info!("PM1 ok: hw={hw} sw={sw} src={src} vbat={vbat}mV vin={vin}mV");
        }
        Err(e) => error!("PM1 init failed: {e:?}"),
    }

    // ---------------------------------------------------------------
    // IO expander + EPD power/reset + NFC power
    // ---------------------------------------------------------------
    let mut ioe = Ioe1::new(RefCellDevice::new(&i2c_bus));
    match ioe.init(&mut delay) {
        Ok(uid) => info!("IOE1 ok: uid=0x{uid:04X} rev={}", ioe.rev().unwrap_or(0)),
        Err(e) => error!("IOE1 init failed: {e:?}"),
    }
    if let Err(e) = board::epd_power_on(&mut ioe, &mut delay) {
        error!("EPD power-on via IOE1 failed: {e:?}");
    }
    if let Err(e) = board::nfc_power(&mut ioe, true) {
        error!("NFC power-on via IOE1 failed: {e:?}");
    }

    // ---------------------------------------------------------------
    // SSD1677 over SPI2
    // ---------------------------------------------------------------
    let spi = Spi::new(p.SPI2, SpiConfig::default().with_frequency(Rate::from_mhz(20)).with_mode(SpiMode::_0))
        .expect("spi config")
        .with_sck(p.GPIO15)
        .with_mosi(p.GPIO14);
    let dc = Output::new(p.GPIO17, Level::Low, OutputConfig::default());
    let cs = Output::new(p.GPIO16, Level::High, OutputConfig::default());
    let busy = Input::new(p.GPIO18, InputConfig::default().with_pull(Pull::Up));
    let mut epd = Ssd1677::new(spi, dc, cs, busy);
    match epd.init(&mut delay) {
        Ok(()) => info!("EPD init ok"),
        Err(e) => error!("EPD init failed: {e:?}"),
    }

    // ---------------------------------------------------------------
    // NFC reader (Pro model only; Lite has no ST25R3916)
    // ---------------------------------------------------------------
    delay.delay_millis(50); // let the ST25R3916 power up
    let mut nfc = St25r3916::new(RefCellDevice::new(&i2c_bus));
    let nfc_ok = match nfc.init(&mut delay) {
        Ok(()) => match nfc.configure_nfca(&mut delay) {
            Ok(()) => {
                let (opc, aux, regd) = nfc.status().unwrap_or((0, 0, 0));
                info!(
                    "NFC ok: identity=0x{:02X} op_ctrl=0x{opc:02X} aux=0x{aux:02X} reg=0x{regd:02X}",
                    nfc.identity().unwrap_or(0)
                );
                true
            }
            Err(e) => {
                error!("NFC configure failed: {e:?}");
                nfc_diagnostics(&mut nfc, &mut delay);
                false
            }
        },
        Err(e) => {
            warn!("NFC init failed (Lite model?): {e:?}");
            false
        }
    };

    // ---------------------------------------------------------------
    // Initial screen
    // ---------------------------------------------------------------
    let fb = FRAMEBUFFER.init(FrameBuffer::new());
    draw_base_screen(fb, nfc_ok);
    match epd.display_gray4(&mut delay, fb, GrayMode::Quality) {
        Ok(()) => info!("EPD initial refresh done"),
        Err(e) => error!("EPD refresh failed: {e:?}"),
    }

    // Front light: brief blink so we know the PM1 PWM path works, then off.
    if let Err(e) = pm1.init_frontlight(5000) {
        warn!("frontlight init failed: {e:?}");
    }
    let _ = pm1.set_frontlight(64);
    delay.delay_millis(300);
    let _ = pm1.set_frontlight(0);

    // ---------------------------------------------------------------
    // Main loop: poll NFC every 200 ms, show new tags on the display.
    // ---------------------------------------------------------------
    let mut last_tag: Option<NfcaTag> = None;
    let mut tag_count: u32 = 0;
    let mut fast_refreshes: u32 = 0;
    let mut last_seen = Instant::now();
    let mut polls: u32 = 0;

    loop {
        delay.delay_millis(200);
        if !nfc_ok {
            continue;
        }
        polls += 1;
        if polls.is_multiple_of(300) {
            let (opc, aux, _) = nfc.status().unwrap_or((0, 0, 0));
            info!("NFC heartbeat: polls={polls} tags={tag_count} op_ctrl=0x{opc:02X} aux=0x{aux:02X}");
        }
        match nfc.nfca_poll(&mut delay) {
            Ok(Some(tag)) => {
                last_seen = Instant::now();
                if last_tag.as_ref().is_some_and(|t| t.uid() == tag.uid()) {
                    let _ = nfc.nfca_halt(&mut delay);
                    continue; // same tag still present
                }
                tag_count += 1;
                info!(
                    "NFC tag #{tag_count}: UID={} ATQA=0x{:04X} SAK=0x{:02X}{}",
                    hex(tag.uid()),
                    tag.atqa,
                    tag.sak,
                    if tag.is_iso14443_4() { " (ISO14443-4)" } else { "" }
                );
                // Try a Type 2 read of block 0..3 for NTAG/Ultralight-class tags.
                let t2 = if !tag.is_iso14443_4() && tag.sak == 0x00 {
                    match nfc.t2t_read(&mut delay, 0) {
                        Ok(d) => {
                            info!("  T2T blocks 0-3: {}", hex(&d));
                            Some(d)
                        }
                        Err(e) => {
                            warn!("  T2T read failed: {e:?}");
                            None
                        }
                    }
                } else {
                    None
                };
                let _ = nfc.nfca_halt(&mut delay);
                draw_tag_panel(fb, &tag, tag_count, t2.as_ref());
                let mode = if fast_refreshes.is_multiple_of(8) { GrayMode::Text } else { GrayMode::Fast };
                fast_refreshes += 1;
                if let Err(e) = epd.display_gray4(&mut delay, fb, mode) {
                    error!("EPD refresh failed: {e:?}");
                }
                last_tag = Some(tag);
            }
            Ok(None) => {
                // Forget the tag once it has been away for a while so a re-tap is reported.
                if last_tag.is_some() && last_seen.elapsed().as_millis() > 1500 {
                    last_tag = None;
                }
            }
            Err(e) => {
                warn!("NFC poll error: {e:?}");
                // Try to recover the field.
                let _ = nfc.field_off();
                delay.delay_millis(20);
                let _ = nfc.configure_nfca(&mut delay);
            }
        }
    }
}

fn nfc_diagnostics<I: embedded_hal::i2c::I2c>(nfc: &mut St25r3916<I>, delay: &mut Delay) {
    let mut regs = [0u8; 64];
    if nfc.dump_regs(&mut regs).is_ok() {
        for (i, chunk) in regs.chunks(16).enumerate() {
            info!("NFC regA 0x{:02X}: {}", i * 16, hex(chunk));
        }
    }
    let mut b = [0u8; 16];
    for (i, r) in [0x05u8, 0x06, 0x0B, 0x0C, 0x0D, 0x0F, 0x15, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x30, 0x31, 0x32, 0x33]
        .iter()
        .enumerate()
    {
        b[i] = nfc.read_reg_b(*r).unwrap_or(0xEE);
    }
    info!("NFC regB [05 06 0B 0C 0D 0F 15 28 29 2A 2B 2C 30 31 32 33]: {}", hex(&b));
    info!("NFC test reg 0x04 = 0x{:02X}", nfc.read_test_reg(0x04).unwrap_or(0xEE));
    info!("NFC amplitude (field supposedly on) = {}", nfc.measure_amplitude(delay).unwrap_or(0));
    match nfc.field_on_ca(delay) {
        Ok(flags) => {
            let (opc, aux, _) = nfc.status().unwrap_or((0, 0, 0));
            info!("NFC field_on_ca: irq=0x{flags:08X} op_ctrl=0x{opc:02X} aux=0x{aux:02X}");
            info!("NFC amplitude after CA = {}", nfc.measure_amplitude(delay).unwrap_or(0));
        }
        Err(e) => warn!("NFC field_on_ca failed: {e:?}"),
    }
    info!("NFC irq now = 0x{:08X}", nfc.read_irq().unwrap_or(0));
}

/// PM1/IOE1 left in 400 kHz mode by an earlier firmware: talk to them at
/// 400 kHz just long enough to clear their SPD bit, then return to 100 kHz.
fn recover_i2c_speed(bus: &RefCell<I2c<'static, esp_hal::Blocking>>, delay: &mut Delay) {
    use paper_name_plate::i2c_reg;
    let fast = I2cConfig::default().with_frequency(Rate::from_khz(400));
    let slow = I2cConfig::default().with_frequency(Rate::from_khz(100));
    {
        let mut b = bus.borrow_mut();
        if let Err(e) = b.apply_config(&fast) {
            warn!("apply_config(400k) failed: {e:?}");
            return;
        }
        let r1 = i2c_reg::write_u8(&mut *b, paper_name_plate::pm1::ADDR, paper_name_plate::pm1::regs::I2C_CFG, 0x00);
        let r2 = i2c_reg::write_u8(&mut *b, paper_name_plate::ioe1::ADDR, paper_name_plate::ioe1::regs::I2C_CFG, 0x00);
        info!("I2C speed recovery: pm1={r1:?} ioe1={r2:?}");
        delay.delay_millis(10);
        let _ = b.apply_config(&slow);
    }
    delay.delay_millis(10);
}

fn hex(bytes: &[u8]) -> String {
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
fn draw_base_screen(fb: &mut FrameBuffer, nfc_ok: bool) {
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
fn draw_tag_panel(fb: &mut FrameBuffer, tag: &NfcaTag, count: u32, t2: Option<&[u8; 16]>) {
    let area = Rectangle::new(Point::new(12, 200), Size::new(WIDTH - 24, 380));
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
