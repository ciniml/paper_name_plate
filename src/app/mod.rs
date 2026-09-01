//! Demo application: NFC-driven info display with touch-controlled front
//! light and an idle deep-sleep policy for the e-paper panel.

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
}

impl App {
    pub fn new(board: Board) -> Self {
        Self {
            board,
            fb: FRAMEBUFFER.init(FrameBuffer::new()),
            displayed: DISPLAYED.init(FrameBuffer::new()),
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
        }
    }

    pub fn run(mut self) -> ! {
        let b = &mut self.board;

        // Initial screen.
        ui::draw_base_screen(self.fb, b.nfc.is_some());
        match b.epd.display_gray4(&mut b.delay, self.fb, GrayMode::Quality) {
            Ok(()) => info!("EPD initial refresh done"),
            Err(e) => error!("EPD refresh failed: {e:?}"),
        }
        self.displayed.copy_from(self.fb);

        // Front light path check: brief blink.
        if let Err(e) = b.pm1.init_frontlight(5000) {
            warn!("frontlight init failed: {e:?}");
        }
        let _ = b.pm1.set_frontlight(64);
        b.delay.delay_ms(300);
        let _ = b.pm1.set_frontlight(0);

        loop {
            let ms = if self.last_tag.is_some() { ACTIVE_POLL_MS } else { IDLE_POLL_MS };
            self.board.delay.delay_ms(ms);
            self.polls += 1;

            self.handle_touch();
            self.handle_buttons();
            self.manage_epd_sleep();
            self.poll_nfc();

            if self.polls.is_multiple_of(60) {
                let (opc, aux) = match self.board.nfc.as_mut() {
                    Some(nfc) => nfc.status().map(|(o, a, _)| (o, a)).unwrap_or((0, 0)),
                    None => (0, 0),
                };
                info!(
                    "heartbeat: up={}s polls={} tags={} epd_sleep={} op_ctrl=0x{opc:02X} aux=0x{aux:02X}",
                    self.boot.elapsed().as_secs(),
                    self.polls,
                    self.tag_count,
                    self.epd_sleeping
                );
            }
        }
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

    /// Button A (GPIO2, active low) toggles the field duty-cycling for A/B tests.
    fn handle_buttons(&mut self) {
        let down = self.board.button_a.is_low();
        if down && !self.btn_a_down {
            self.field_always_on = !self.field_always_on;
            info!("Button A: field_always_on={}", self.field_always_on);
            if self.field_always_on
                && let Some(nfc) = self.board.nfc.as_mut()
            {
                let _ = nfc.field_on(&mut self.board.delay);
            }
        }
        self.btn_a_down = down;
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
