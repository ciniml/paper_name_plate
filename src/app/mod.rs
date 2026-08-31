//! Demo application: NFC-driven info display with touch-controlled front
//! light and an idle deep-sleep policy for the e-paper panel.

pub mod ui;

use embedded_hal::delay::DelayNs;
use esp_hal::time::Instant;
use log::{error, info, warn};
use static_cell::StaticCell;

use crate::board::{self, Board};
use crate::ft6336::TouchPoint;
use crate::ssd1677::{FrameBuffer, GrayMode};
use crate::st25r3916::{Error as NfcError, NfcaTag};

/// EPD is put into deep sleep after this much time without activity.
const EPD_IDLE_SLEEP_MS: u64 = 60_000;
/// A full (Text-mode) refresh is forced after this many fastest updates.
const FASTEST_PER_FULL: u32 = 10;
/// A tag is considered gone after not answering for this long.
const TAG_FORGET_MS: u64 = 1_500;

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
    polls: u32,
    epd_sleeping: bool,
    frontlight_on: bool,
    touch_down: bool,
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
            polls: 0,
            epd_sleeping: false,
            frontlight_on: false,
            touch_down: false,
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
            let ms = if self.epd_sleeping { 500 } else { 200 };
            self.board.delay.delay_ms(ms);
            self.polls += 1;

            self.handle_touch();
            self.manage_epd_sleep();
            self.poll_nfc();

            if self.polls.is_multiple_of(300)
                && let Some(nfc) = self.board.nfc.as_mut()
            {
                let (opc, aux, _) = nfc.status().unwrap_or((0, 0, 0));
                info!("heartbeat: polls={} tags={} op_ctrl=0x{opc:02X} aux=0x{aux:02X}", self.polls, self.tag_count);
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
        match nfc.nfca_poll(&mut b.delay) {
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
        // Type 2 read for NTAG/Ultralight-class tags.
        let t2 = if !tag.is_iso14443_4() && tag.sak == 0x00 {
            match nfc.t2t_read(&mut b.delay, 0) {
                Ok(d) => {
                    info!("  T2T blocks 0-3: {}", ui::hex(&d));
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
        let _ = nfc.nfca_halt(&mut b.delay);

        ui::draw_tag_panel(self.fb, &tag, self.tag_count, t2.as_ref());
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
