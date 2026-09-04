//! Demo application: NFC-driven info display with touch-controlled front
//! light and an idle deep-sleep policy for the e-paper panel.

pub mod ble;
pub mod plate;
pub mod ui;

use embedded_hal::delay::DelayNs;
use esp_hal::time::Instant;
use log::{error, info, warn};
use static_cell::StaticCell;

use crate::board::{self, Board};
use crate::ft6336::TouchPoint;
use crate::isodep::IsoDep;
use crate::ndef;
use crate::ssd1677::{FrameBuffer, GrayMode};
use crate::st25r3916::{Error as NfcError, NfcaTag};
use crate::t2t_emu::{Event as EmuEvent, T2tEmulator};
use plate::{MonoImage, PlateContent};

/// EPD is put into deep sleep after this much time without activity.
const EPD_IDLE_SLEEP_MS: u64 = 60_000;
/// A full (Text-mode) refresh is forced after this many fastest updates.
const FASTEST_PER_FULL: u32 = 10;
/// A tag is considered gone after not answering for this long.
const TAG_FORGET_MS: u64 = 1_500;
/// With no tag present the RF field is only pulsed on for each poll
/// (duty cycling); polling then slows down to this period.
const IDLE_POLL_MS: u32 = 500;
/// Poll period while a tag is (or was just) present.
const ACTIVE_POLL_MS: u32 = 200;
/// After switching the field on, give phones (card emulation needs to boot)
/// this long before the first WUPA. Physical tags would need ~5 ms.
const FIELD_SETTLE_MS: u32 = 80;

static FRAMEBUFFER: StaticCell<FrameBuffer> = StaticCell::new();
/// Copy of what is currently on the panel (baseline for differential updates).
static DISPLAYED: StaticCell<FrameBuffer> = StaticCell::new();

pub struct App {
    board: Board,
    fb: &'static mut FrameBuffer,
    displayed: &'static mut FrameBuffer,
    last_tag: Option<NfcaTag>,
    tag_count: u32,
    fast_refreshes: u32,
    last_activity: Instant,
    boot: Instant,
    polls: u32,
    epd_sleeping: bool,
    frontlight_on: bool,
    touch_down: bool,
    /// Button A toggles: `true` = field always on (reference behaviour),
    /// `false` = duty-cycled field when no tag is present.
    field_always_on: bool,
    btn_a_down: bool,
    btn_a_since: Instant,
    btn_a_armed: bool,
    /// Diagnostic: alternate emulation/reader every 10 s automatically.
    auto_ab: bool,
    auto_ab_since: Instant,
    /// `true`: act as an NFC tag (name-plate mode); `false`: reader mode.
    emulate: bool,
    emu: T2tEmulator,
    emu_running: bool,
    /// Last time the emulator saw any field activity (for the re-init watchdog).
    emu_last_field: Instant,
    content: PlateContent,
    /// A display image is stored in the flash image slot. Large images are
    /// not kept in `content.image` between redraws (see [`Self::draw_plate`]).
    image_in_flash: bool,
}

/// Images up to this many bytes stay resident in `content.image`; larger
/// ones (a full screen is 48 KB) are re-read from flash for each redraw so
/// that the heap can hold an incoming BLE image alongside the BLE stack.
const IMAGE_KEEP_MAX: usize = 8192;

impl App {
    pub fn new(board: Board) -> Self {
        Self {
            board,
            fb: FrameBuffer::init_uninit(FRAMEBUFFER.uninit()),
            displayed: FrameBuffer::init_uninit(DISPLAYED.uninit()),
            last_tag: None,
            tag_count: 0,
            fast_refreshes: 0,
            last_activity: Instant::now(),
            boot: Instant::now(),
            polls: 0,
            epd_sleeping: false,
            frontlight_on: false,
            touch_down: false,
            field_always_on: false,
            btn_a_down: false,
            btn_a_since: Instant::now(),
            btn_a_armed: false,
            auto_ab: false,
            auto_ab_since: Instant::now(),
            emulate: true,
            emu: T2tEmulator::new([0x04, 0x50, 0x41, 0x50, 0x45, 0x52, 0x01]),
            emu_running: false,
            emu_last_field: Instant::now(),
            content: PlateContent::demo(),
            image_in_flash: false,
        }
    }

    /// Render the plate into `fb`, temporarily loading a large image from
    /// flash if it is not resident.
    fn draw_plate(&mut self) {
        let load = self.content.image.is_none() && self.image_in_flash;
        if load {
            self.content.image =
                crate::config_store::load_image(&mut self.board.flash).and_then(MonoImage::decode_owned);
        }
        plate::draw(self.fb, &self.content);
        if load {
            self.content.image = None;
        }
    }

    /// Drop a large image from memory once it is drawn and persisted.
    fn trim_image(&mut self) {
        if self.image_in_flash
            && self.content.image.as_ref().is_some_and(|i| i.bits.len() > IMAGE_KEEP_MAX)
        {
            self.content.image = None;
        }
    }

    pub fn run(mut self) -> ! {
        let b = &mut self.board;

        // Restore persisted content, if any.
        if let Some(stored) = crate::config_store::load(&mut b.flash) {
            info!("config: restored {} B NDEF from flash", stored.len());
            self.content.apply_ndef(&stored);
        }
        if let Some(img) = crate::config_store::load_image(&mut b.flash).and_then(MonoImage::decode_owned) {
            info!("config: restored {}x{} image from flash", img.width, img.height);
            self.content.image = Some(img);
            self.image_in_flash = true;
        }

        // Initial screen: the name plate itself.
        plate::draw(self.fb, &self.content);
        self.trim_image();
        let b = &mut self.board;
        match b.epd.display_gray4(&mut b.delay, self.fb, GrayMode::Quality) {
            Ok(()) => info!("EPD initial refresh done"),
            Err(e) => error!("EPD refresh failed: {e:?}"),
        }
        self.displayed.copy_from(self.fb);

        // Serve the plate content over NFC.
        self.emu.set_ndef(&self.content.to_ndef());

        // Front light path check: brief blink.
        if let Err(e) = b.pm1.init_frontlight(5000) {
            warn!("frontlight init failed: {e:?}");
        }
        let _ = b.pm1.set_frontlight(64);
        b.delay.delay_ms(300);
        let _ = b.pm1.set_frontlight(0);

        self.main_loop()
    }

    /// Dispatch to the BLE-enabled main loop, or the plain one if the radio
    /// could not be brought up.
    fn main_loop(&mut self) -> ! {
        use esp_radio::ble::controller::BleConnector;
        let connector = self.board.bt.take().and_then(|bt| match BleConnector::new(bt, Default::default()) {
            Ok(c) => Some(c),
            Err(e) => {
                error!("BLE connector init failed: {e:?}");
                None
            }
        });
        match connector {
            Some(c) => {
                let hci = bleps::HciConnector::new(ble::BufferedHci::new(c), ble::millis);
                info!("BLE ready (device name: {})", ble::DEVICE_NAME);
                loop {
                    self.ble_session(&hci);
                }
            }
            None => loop {
                self.tick(true, 20);
            },
        }
    }

    /// One iteration of the application work (touch, buttons, EPD sleep,
    /// NFC emulation/reader, heartbeat). `allow_emulation == false` skips the
    /// time-consuming NFC slice (used while BLE transfers are in flight).
    fn tick(&mut self, allow_emulation: bool, emu_ms: u32) {
        if self.emulate {
            if allow_emulation {
                self.run_emulation_slice(emu_ms);
            } else {
                self.board.delay.delay_ms(1);
            }
            self.polls += 1;
        } else {
            let ms = if self.last_tag.is_some() { ACTIVE_POLL_MS } else { IDLE_POLL_MS };
            self.board.delay.delay_ms(ms);
            self.polls += 1;
            self.poll_nfc();
        }

        self.handle_touch();
        self.handle_buttons();
        if self.auto_ab && self.auto_ab_since.elapsed().as_millis() > 10_000 {
            self.auto_ab_since = Instant::now();
            self.emulate = !self.emulate;
            info!("AUTO A/B: emulate={}", self.emulate);
            if !self.emulate {
                self.stop_emulation();
                if let Some(nfc) = self.board.nfc.as_mut() {
                    let _ = nfc.configure_nfca(&mut self.board.delay);
                }
            }
        }
        self.manage_epd_sleep();

        if self.polls.is_multiple_of(300)
            && let Some(nfc) = self.board.nfc.as_mut()
        {
            let (opc, aux) = nfc.status().map(|(o, a, _)| (o, a)).unwrap_or((0, 0));
            info!(
                "heartbeat: up={}s polls={} tags={} epd_sleep={} emu={:?} op_ctrl=0x{opc:02X} aux=0x{aux:02X}",
                self.boot.elapsed().as_secs(),
                self.polls,
                self.tag_count,
                self.epd_sleeping,
                self.emu.state()
            );
        }
    }

    /// A complete image arrived over BLE: persist, adopt and redraw.
    pub(crate) fn on_ble_image(&mut self, img: MonoImage) {
        info!("BLE image applied: {}x{} ({} B)", img.width, img.height, img.bits.len());
        // Free the previous image first: with the BLE stack loaded there is
        // not enough heap for two full-screen images plus staging copies.
        self.content.image = None;
        log::debug!("heap: {}", esp_alloc::HEAP.stats());
        match crate::config_store::save_image(&mut self.board.flash, img.width, img.height, &img.bits) {
            Ok(()) => self.image_in_flash = true,
            Err(e) => error!("image save failed: {e:?}"),
        }
        self.content.image = Some(img);
        let canonical = self.content.to_ndef();
        self.emu.set_ndef(&canonical);
        let _ = crate::config_store::save(&mut self.board.flash, &canonical);
        self.wake_epd();
        plate::draw(self.fb, &self.content);
        self.trim_image();
        let b = &mut self.board;
        match b.epd.display_gray4(&mut b.delay, self.fb, GrayMode::Quality) {
            Ok(()) => self.displayed.copy_from(self.fb),
            Err(e) => error!("EPD refresh failed: {e:?}"),
        }
        self.fast_refreshes = 1;
        self.last_activity = Instant::now();
    }

    /// A tap toggles the front light and counts as activity.
    fn handle_touch(&mut self) {
        let b = &mut self.board;
        let Some(touch) = b.touch.as_mut() else { return };
        let mut pts = [TouchPoint::default(); 2];
        match touch.read(&mut pts) {
            Ok(n) if n > 0 => {
                if !self.touch_down {
                    self.touch_down = true;
                    self.frontlight_on = !self.frontlight_on;
                    info!("Touch at ({}, {}) -> frontlight {}", pts[0].x, pts[0].y, self.frontlight_on);
                    let _ = b.pm1.set_frontlight(if self.frontlight_on { 128 } else { 0 });
                    self.last_activity = Instant::now();
                }
            }
            Ok(_) => self.touch_down = false,
            Err(e) => log::debug!("touch read: {e:?}"),
        }
    }

    /// Button A (GPIO2, active low) switches between tag emulation and reader
    /// mode. Requires ~1 s hold so it cannot be toggled by accident while
    /// handling the device.
    fn handle_buttons(&mut self) {
        let down = self.board.button_a.is_low();
        if down && !self.btn_a_down {
            self.btn_a_since = Instant::now();
            self.btn_a_armed = true;
        }
        if down && self.btn_a_down && self.btn_a_armed && self.btn_a_since.elapsed().as_millis() > 1000 {
            self.btn_a_armed = false; // once per press
            self.emulate = !self.emulate;
            info!("Button A (held): emulate={}", self.emulate);
            if !self.emulate {
                self.stop_emulation();
                if let Some(nfc) = self.board.nfc.as_mut() {
                    let _ = nfc.configure_nfca(&mut self.board.delay);
                }
            }
        }
        self.btn_a_down = down;
    }

    fn stop_emulation(&mut self) {
        if self.emu_running
            && let Some(nfc) = self.board.nfc.as_mut()
        {
            let _ = self.emu.stop(nfc);
            self.emu_running = false;
        }
    }

    /// Run the tag emulator for roughly `ms` milliseconds.
    fn run_emulation_slice(&mut self, ms: u32) {
        let b = &mut self.board;
        let Some(nfc) = b.nfc.as_mut() else {
            b.delay.delay_ms(ms);
            return;
        };
        if !self.emu_running {
            let _ = nfc.field_off();
            match self.emu.start(nfc, &mut b.delay) {
                Ok(()) => {
                    info!("T2T emulation started, UID={}", ui::hex(&self.emu.uid()));
                    self.emu_running = true;
                }
                Err(e) => {
                    error!("T2T emulation start failed: {e:?}");
                    b.delay.delay_ms(1000);
                    return;
                }
            }
        }
        // Watchdog: if the reader field has not been seen for a long time the
        // chip may be wedged in a way we cannot observe; re-init it.
        if self.emu_last_field.elapsed().as_millis() > 120_000 {
            info!("T2T: no field for 120 s -> re-init");
            self.emu_last_field = Instant::now();
            self.emu_running = false;
            return;
        }
        let t0 = Instant::now();
        while t0.elapsed().as_millis() < ms as u64 {
            match self.emu.update(nfc, &mut b.delay) {
                Ok(EmuEvent::None) => b.delay.delay_ms(1),
                Ok(EmuEvent::FieldOn) => {
                    self.emu_last_field = Instant::now();
                    info!("T2T: field on (up={}s)", self.boot.elapsed().as_secs());
                }
                Ok(EmuEvent::Selected) => {
                    info!("T2T: selected by reader");
                    self.last_activity = Instant::now();
                    self.emu_last_field = Instant::now();
                }
                Ok(EmuEvent::FieldOff { written }) => {
                    self.emu_last_field = Instant::now();
                    info!("T2T: field off, {} commands, last=0x{:02X}, written={written}", self.emu.commands, self.emu.last_cmd);
                    if written {
                        self.on_tag_written();
                        return;
                    }
                }
                Err(e) => {
                    warn!("T2T update error: {e:?}");
                    self.emu_running = false;
                    return;
                }
            }
        }
    }

    /// The phone wrote to our emulated tag: adopt the new content and
    /// re-render the plate.
    fn on_tag_written(&mut self) {
        if let Some(m) = self.emu.ndef() {
            info!("T2T: new NDEF ({} B): {}", m.len(), ndef::summarize(m));
            let m: alloc::vec::Vec<u8> = m.into();
            self.content.apply_ndef(&m);
        } else {
            info!("T2T: NDEF erased; keeping current content");
        }
        // Normalise what we serve (canonical record layout) and persist it.
        let canonical = self.content.to_ndef();
        self.emu.set_ndef(&canonical);
        match crate::config_store::save(&mut self.board.flash, &canonical) {
            Ok(()) => info!("config: saved {} B NDEF to flash", canonical.len()),
            Err(e) => error!("config: save failed: {e:?}"),
        }
        // A (small) image delivered over NFC also goes to the image slot so
        // the flash copy never disagrees with what is shown.
        if let Some(img) = &self.content.image
            && img.bits.len() <= IMAGE_KEEP_MAX
        {
            match crate::config_store::save_image(&mut self.board.flash, img.width, img.height, &img.bits) {
                Ok(()) => self.image_in_flash = true,
                Err(e) => error!("image save failed: {e:?}"),
            }
        }
        self.wake_epd();
        self.draw_plate();
        let b = &mut self.board;
        match b.epd.display_gray4(&mut b.delay, self.fb, GrayMode::Text) {
            Ok(()) => self.displayed.copy_from(self.fb),
            Err(e) => error!("EPD refresh failed: {e:?}"),
        }
        self.fast_refreshes = 1;
    }

    fn manage_epd_sleep(&mut self) {
        if !self.epd_sleeping && self.last_activity.elapsed().as_millis() > EPD_IDLE_SLEEP_MS {
            info!("EPD: idle -> deep sleep");
            let b = &mut self.board;
            if let Err(e) = b.epd.deep_sleep(&mut b.delay) {
                error!("EPD deep sleep failed: {e:?}");
            }
            self.epd_sleeping = true;
        }
    }

    fn wake_epd(&mut self) {
        if !self.epd_sleeping {
            return;
        }
        info!("EPD: waking from deep sleep");
        let b = &mut self.board;
        let _ = board::epd_hard_reset(&mut b.ioe, &mut b.delay);
        if let Err(e) = b.epd.init(&mut b.delay) {
            error!("EPD re-init failed: {e:?}");
        }
        self.epd_sleeping = false;
        self.fast_refreshes = 0;
    }

    fn poll_nfc(&mut self) {
        let b = &mut self.board;
        let Some(nfc) = b.nfc.as_mut() else { return };
        // Duty-cycle the RF field: while no tag is around the field is only
        // on during the poll itself (the ~100 mA TX driver dominates the
        // board's idle power draw).
        let duty_cycling = self.last_tag.is_none() && !self.field_always_on;
        if duty_cycling {
            if nfc.field_on(&mut b.delay).is_err() {
                return;
            }
            b.delay.delay_ms(FIELD_SETTLE_MS);
        }
        let result = nfc.nfca_poll(&mut b.delay);
        // Diagnostics: any RF activity short of a full detection.
        let flags = nfc.last_request_irq;
        // RXS | RXE | COL | error bits, i.e. a tag answered but activation failed.
        if flags & 0x3400_F000 != 0 && !matches!(result, Ok(Some(_))) {
            info!("NFC activity: irq=0x{flags:08X} duty={duty_cycling} result={result:?}");
        }
        if duty_cycling && matches!(result, Ok(None)) {
            let _ = nfc.field_off();
        }
        match result {
            Ok(Some(tag)) => {
                self.last_activity = Instant::now();
                let same = self.last_tag.as_ref().is_some_and(|t| {
                    // Random UIDs (smartphones) change on every activation:
                    // treat a continuously-present random-UID tag as the same.
                    t.uid() == tag.uid() || (t.has_random_uid() && tag.has_random_uid())
                });
                if same {
                    let _ = nfc.nfca_halt(&mut b.delay);
                    return;
                }
                self.on_new_tag(tag);
            }
            Ok(None) => {
                if self.last_tag.is_some() && self.last_activity.elapsed().as_millis() > TAG_FORGET_MS {
                    self.last_tag = None;
                }
            }
            Err(NfcError::I2c(e)) => {
                warn!("NFC poll I2C error: {e:?}; reconfiguring");
                let _ = nfc.field_off();
                b.delay.delay_ms(20);
                let _ = nfc.configure_nfca(&mut b.delay);
            }
            Err(e) => {
                // Transient CRC/parity/timeout errors are normal while a tag
                // enters or leaves the field.
                log::debug!("NFC poll: {e:?}");
            }
        }
    }

    fn on_new_tag(&mut self, tag: NfcaTag) {
        self.wake_epd();
        self.tag_count += 1;
        let b = &mut self.board;
        let nfc = b.nfc.as_mut().expect("poll_nfc only runs with NFC");
        info!(
            "NFC tag #{}: UID={} ATQA=0x{:04X} SAK=0x{:02X}{}",
            self.tag_count,
            ui::hex(tag.uid()),
            tag.atqa,
            tag.sak,
            if tag.is_iso14443_4() { " (ISO14443-4)" } else { "" }
        );
        // Content: NDEF over ISO-DEP for phones/Type 4, raw blocks for Type 2.
        let mut t2 = None;
        let mut detail: Option<alloc::string::String> = None;
        if tag.is_iso14443_4() {
            match IsoDep::activate(nfc, &mut b.delay) {
                Ok(mut dep) => {
                    info!("  ATS: {} (FWT {} ms, FSC {})", ui::hex(&dep.ats().raw), dep.ats().fwt_ms, dep.ats().fsc);
                    match ndef::read_type4(&mut dep, nfc, &mut b.delay) {
                        Ok(msg) => {
                            let summary = ndef::summarize(&msg);
                            info!("  NDEF ({} B): {summary}", msg.len());
                            detail = Some(summary);
                        }
                        Err(e) => {
                            info!("  NDEF read: {e:?}");
                            detail = Some(match e {
                                ndef::Error::Status { step, sw } => {
                                    let mut s = alloc::string::String::new();
                                    use core::fmt::Write as _;
                                    let _ = write!(s, "no NDEF ({step}: SW={sw:04X})");
                                    s
                                }
                                _ => alloc::string::String::from("no NDEF (protocol error)"),
                            });
                        }
                    }
                    let _ = dep.deselect(nfc, &mut b.delay);
                }
                Err(e) => {
                    warn!("  RATS failed: {e:?}");
                    let _ = nfc.nfca_halt(&mut b.delay);
                }
            }
        } else {
            if tag.sak == 0x00 {
                match nfc.t2t_read(&mut b.delay, 0) {
                    Ok(d) => {
                        info!("  T2T blocks 0-3: {}", ui::hex(&d));
                        t2 = Some(d);
                    }
                    Err(e) => warn!("  T2T read failed: {e:?}"),
                }
            }
            let _ = nfc.nfca_halt(&mut b.delay);
        }

        ui::draw_tag_panel(self.fb, &tag, self.tag_count, t2.as_ref());
        if let Some(d) = detail.as_deref() {
            ui::draw_tag_detail(self.fb, d);
        }
        self.refresh_panel_region();
        self.last_tag = Some(tag);
    }

    /// Refresh the tag panel: differential fastest update most of the time,
    /// with a periodic absolute Text refresh against ghosting.
    fn refresh_panel_region(&mut self) {
        let b = &mut self.board;
        if self.fast_refreshes.is_multiple_of(FASTEST_PER_FULL) {
            match b.epd.display_gray4(&mut b.delay, self.fb, GrayMode::Text) {
                Ok(()) => self.displayed.copy_from(self.fb),
                Err(e) => error!("EPD text refresh failed: {e:?}"),
            }
        } else {
            let (x0, w, y0, h) = FrameBuffer::native_region(ui::PANEL_X, ui::PANEL_Y, ui::PANEL_W, ui::PANEL_H);
            let t0 = Instant::now();
            match b.epd.refresh_fastest(&mut b.delay, &self.fb.msb, &self.displayed.lsb, &self.displayed.msb, x0, w, y0, h) {
                Ok(()) => {
                    self.displayed.apply_mono_region(&self.fb.msb, x0, w, y0, h);
                    info!("EPD fastest refresh: {} ms", t0.elapsed().as_millis());
                }
                Err(e) => error!("EPD fastest refresh failed: {e:?}"),
            }
        }
        self.fast_refreshes += 1;
    }
}
