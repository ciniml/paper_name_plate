//! BLE image transfer: a GATT service that receives a 1-bpp image.
//!
//! Protocol (all writes, little endian):
//! * CTRL characteristic:
//!   `01 w:u16 h:u16 len:u32` — start a transfer (`len` = row_bytes*h)
//!   `02` — commit (apply if all bytes arrived), `03` — abort
//! * DATA characteristic: sequential payload chunks
//! * STATUS characteristic (read): `state:u8 received:u32 expected:u32`
//!   (state: 0 idle, 1 receiving, 2 complete)
//!
//! The BLE session runs interleaved with the normal app tick; while a
//! transfer is in flight the NFC emulation slice is skipped so the HCI
//! queue drains quickly.

use alloc::vec::Vec;
use core::cell::RefCell;

use embedded_hal::delay::DelayNs;

use bleps::ad_structure::{
    create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
};
use bleps::attribute_server::{AttributeServer, WorkResult};
use bleps::no_rng::NoRng;
use bleps::{gatt, Ble, HciConnector};
use esp_radio::ble::controller::BleConnector;
use log::{error, info, warn};

use super::plate::MonoImage;

/// bleps reads HCI **one byte at a time**; if the underlying transport
/// discards the rest of a packet on a short read, the stream desyncs
/// ("Expected async data"). This adapter always pulls whole packets from the
/// controller into a local buffer and serves them out byte-wise.
pub struct BufferedHci {
    inner: BleConnector<'static>,
    buf: [u8; 1024],
    start: usize,
    end: usize,
}

impl BufferedHci {
    pub fn new(inner: BleConnector<'static>) -> Self {
        Self { inner, buf: [0; 1024], start: 0, end: 0 }
    }
}

impl embedded_io_06::ErrorType for BufferedHci {
    type Error = esp_radio::ble::controller::BleConnectorError;
}

impl embedded_io_06::Read for BufferedHci {
    fn read(&mut self, out: &mut [u8]) -> Result<usize, Self::Error> {
        if self.start == self.end {
            self.start = 0;
            self.end = self.inner.read(&mut self.buf)?;
        }
        let n = out.len().min(self.end - self.start);
        out[..n].copy_from_slice(&self.buf[self.start..self.start + n]);
        self.start += n;
        Ok(n)
    }
}

impl embedded_io_06::Write for BufferedHci {
    fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
        self.inner.write(data)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

pub const DEVICE_NAME: &str = "PaperPlate";
const MAX_IMAGE_BYTES: usize = 48_000; // 480x800 / 8

pub fn millis() -> u64 {
    esp_hal::time::Instant::now().duration_since_epoch().as_millis()
}

#[derive(Default)]
pub struct ImgRx {
    receiving: bool,
    width: u16,
    height: u16,
    expected: usize,
    data: Vec<u8>,
    done: Option<MonoImage>,
    /// millis() of the last GATT interaction (for BLE-priority scheduling).
    last_ms: u64,
}

impl ImgRx {
    fn ctrl(&mut self, d: &[u8]) {
        self.last_ms = millis();
        match d.first() {
            Some(0x01) if d.len() >= 9 => {
                let w = u16::from_le_bytes([d[1], d[2]]);
                let h = u16::from_le_bytes([d[3], d[4]]);
                let len = u32::from_le_bytes([d[5], d[6], d[7], d[8]]) as usize;
                let row_bytes = (w as usize).div_ceil(8);
                if w == 0 || h == 0 || w > 480 || h > 800 || len != row_bytes * h as usize || len > MAX_IMAGE_BYTES {
                    warn!("BLE img: bad start {w}x{h} len={len}");
                    self.reset();
                    return;
                }
                info!("BLE img: start {w}x{h} ({len} B)");
                self.receiving = true;
                self.width = w;
                self.height = h;
                self.expected = len;
                self.data = Vec::with_capacity(len);
                self.done = None;
            }
            Some(0x02) => {
                if self.receiving && self.data.len() == self.expected {
                    info!("BLE img: commit ({} B)", self.data.len());
                    self.done = Some(MonoImage {
                        width: self.width,
                        height: self.height,
                        bits: core::mem::take(&mut self.data),
                    });
                } else {
                    warn!("BLE img: commit with {}/{} B", self.data.len(), self.expected);
                }
                self.receiving = false;
            }
            Some(0x03) => {
                info!("BLE img: abort");
                self.reset();
            }
            _ => warn!("BLE img: unknown ctrl {d:02X?}"),
        }
    }

    fn push(&mut self, d: &[u8]) {
        self.last_ms = millis();
        if self.receiving && self.data.len() + d.len() <= self.expected {
            let before = self.data.len();
            self.data.extend_from_slice(d);
            // Throttled progress (every ~10%).
            let step = (self.expected / 10).max(1);
            if before / step != self.data.len() / step {
                log::info!("BLE img: {}/{}", self.data.len(), self.expected);
            }
        } else {
            log::warn!("BLE img: push dropped (recv={} have={} +{} exp={})", self.receiving, self.data.len(), d.len(), self.expected);
        }
    }

    fn status(&mut self, out: &mut [u8]) -> usize {
        self.last_ms = millis();
        if out.len() < 9 {
            return 0;
        }
        out[0] = if self.done.is_some() {
            2
        } else if self.receiving {
            1
        } else {
            0
        };
        out[1..5].copy_from_slice(&(self.data.len() as u32).to_le_bytes());
        out[5..9].copy_from_slice(&(self.expected as u32).to_le_bytes());
        9
    }

    fn reset(&mut self) {
        *self = Self::default();
    }
}

impl super::App {
    /// One BLE session: advertise, serve GATT until disconnect (or error),
    /// running the normal app tick in between. Returns to be called again.
    pub(crate) fn ble_session(&mut self, hci: &HciConnector<BufferedHci>) {
        let mut ble = Ble::new(hci);
        if let Err(e) = ble.init() {
            error!("BLE stack init failed: {e:?}");
            self.board.delay.delay_ms(1000);
            return;
        }
        let _ = ble.cmd_set_le_advertising_parameters();
        match create_advertising_data(&[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(DEVICE_NAME),
        ]) {
            Ok(data) => {
                let _ = ble.cmd_set_le_advertising_data(data);
            }
            Err(e) => {
                error!("BLE adv data failed: {e:?}");
                return;
            }
        }
        let _ = ble.cmd_set_le_advertise_enable(true);
        log::debug!("BLE advertising as {DEVICE_NAME}");

        let rx = RefCell::new(ImgRx::default());
        let mut wf_ctrl = |_offset: usize, data: &[u8]| {
            rx.borrow_mut().ctrl(data);
        };
        let mut wf_data = |_offset: usize, data: &[u8]| {
            rx.borrow_mut().push(data);
        };
        let mut rf_status = |_offset: usize, data: &mut [u8]| -> usize { rx.borrow_mut().status(data) };

        gatt!([service {
            uuid: "50415045-5250-4c41-5445-000000000001",
            characteristics: [
                characteristic {
                    uuid: "50415045-5250-4c41-5445-000000000002",
                    write: wf_ctrl,
                },
                characteristic {
                    uuid: "50415045-5250-4c41-5445-000000000003",
                    write: wf_data,
                },
                characteristic {
                    uuid: "50415045-5250-4c41-5445-000000000004",
                    read: rf_status,
                },
            ],
        },]);

        let mut rng = NoRng;
        let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes, &mut rng);

        let mut errors = 0u32;
        loop {
            // Pump HCI hard so advertising stays continuous and a connected
            // central is serviced without the multi-ms stalls that make GATT
            // writes fail.
            for _ in 0..96 {
                match srv.do_work() {
                    Ok(WorkResult::GotDisconnected) => {
                        info!("BLE: disconnected");
                        return;
                    }
                    Ok(WorkResult::DidWork) => {}
                    Err(e) => {
                        errors += 1;
                        if errors >= 8 {
                            warn!("BLE: {errors} work errors ({e:?}); restarting session");
                            return;
                        }
                        break;
                    }
                }
            }

            if let Some(img) = rx.borrow_mut().done.take() {
                self.on_ble_image(img);
                errors = 0;
            }

            // Give the rest of the app a slice only when BLE is idle; while a
            // central is active the slow NFC-emulation service would starve the
            // link. Touch/buttons/heartbeat still run every iteration.
            let ble_active = rx.borrow().receiving || millis().saturating_sub(rx.borrow().last_ms) < 3000;
            if ble_active {
                self.tick_light();
            } else {
                // Keep the BLE service cadence tight so advertising stays
                // continuous (discoverable) while NFC emulation still runs.
                self.tick(true, 6);
            }
        }
    }

    /// Cheap housekeeping used while a BLE transfer is in progress: touch,
    /// buttons and the sleep/heartbeat bookkeeping, but no NFC emulation.
    fn tick_light(&mut self) {
        self.handle_touch();
        self.handle_buttons();
        self.manage_epd_sleep();
        self.board.delay.delay_ms(1);
    }
}
