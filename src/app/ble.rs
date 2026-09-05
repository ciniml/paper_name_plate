//! BLE image transfer: a GATT service that receives a 1-bpp image.
//!
//! Protocol v2 (all little endian). Windowed, acknowledged transfer:
//!
//! * CTRL characteristic (write / write-without-response):
//!   * `11 len:u32 crc32:u32 ack_every:u8` — start a plate-text transfer
//!     (UTF-8 `name\ntitle\norg\nnote\nurl`, applied on commit).
//!   * `10 w:u16 h:u16 len:u32 crc32:u32 ack_every:u8` — start a transfer
//!     (`len` = row_bytes*h, `crc32` = IEEE CRC-32 of the payload,
//!     `ack_every` = number of accepted DATA packets per acknowledgement,
//!     0 → 8).
//!   * `02` — commit: verify length + CRC and apply.
//!   * `03` — abort.
//!   * `04` — sync: request a STATUS notification now.
//! * DATA characteristic (write-without-response): `off:u16 | payload`.
//!   Packets must arrive in order (`off` == bytes received so far); an
//!   out-of-order packet is dropped and answered with a STATUS notification
//!   (NAK) carrying the offset the device actually expects.
//! * CONTENT characteristic (read): current plate text in the plain form.
//! * STATUS characteristic (read / notify): `state:u8 received:u32 expected:u32`
//!   state: 0 idle, 1 receiving, 2 committed, 3 CRC error, 4 commit with
//!   missing data. A notification is sent after every `ack_every` accepted
//!   packets, when the last byte arrives, on NAK, and after start/commit/
//!   abort/sync.
//!
//! The client sends `ack_every` packets, waits for the STATUS notification
//! whose `received` covers them, and continues (rewinding to `received` on a
//! NAK). This gives back-pressure so the phone's write-without-response
//! flood can never overrun the device, and every loss is detected and
//! repaired within one window.
//!
//! The GATT table is built by hand (not with the `gatt!` macro) so the DATA
//! and CTRL characteristics can declare the Write-Without-Response property
//! (0x04); Android/Chrome refuses `writeValueWithoutResponse` otherwise.
//!
//! The BLE session runs interleaved with the normal app tick; while a
//! transfer is in flight nothing but the HCI pump runs.

use alloc::vec::Vec;
use core::cell::RefCell;

use embedded_hal::delay::DelayNs;

use bleps::ad_structure::{
    create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
};
use bleps::att::Uuid;
use bleps::attribute::Attribute;
use bleps::attribute_server::{
    AttributeServer, NotificationData, WorkResult, CHARACTERISTIC_UUID16, PRIMARY_SERVICE_UUID16,
};
use bleps::no_rng::NoRng;
use bleps::{
    AdvertisingFilterPolicy, AdvertisingParameters, AdvertisingType, Ble, HciConnector, OwnAddressType,
    PeerAddressType,
};
use esp_radio::ble::controller::BleConnector;
use log::{error, info, warn};

use super::plate::MonoImage;

/// HCI transport adapter between the esp-radio controller and bleps.
///
/// bleps reads HCI **one byte at a time** and expects every byte of a
/// packet to be readable the instant it asks for it (it `unwrap()`s the
/// reads inside a packet). Two things break that assumption with the raw
/// [`BleConnector`] byte stream:
///
/// * `BleConnector::read` concatenates queued packets and may split one
///   across two reads, so a short read desyncs the stream ("Expected async
///   data").
/// * A large ATT write (MTU 128) arrives as one L2CAP PDU split across
///   several HCI ACL fragments (first + continuing). bleps reassembles them
///   but panics if the continuation has not reached the host yet.
///
/// So this adapter pulls whole packets with `BleConnector::next`, waits for
/// (and merges) ACL continuation fragments itself, and serves the finished
/// packet byte-wise. Any other packet that arrives in between the fragments
/// is queued behind the merged one.
pub struct BufferedHci {
    inner: BleConnector<'static>,
    buf: [u8; 1024],
    start: usize,
    end: usize,
}

const HCI_ACL: u8 = 0x02;
const ACL_HDR: usize = 5; // type + handle/flags + length

impl BufferedHci {
    pub fn new(inner: BleConnector<'static>) -> Self {
        Self { inner, buf: [0; 1024], start: 0, end: 0 }
    }

    fn acl_pb_flag(pkt: &[u8]) -> u8 {
        (pkt[2] >> 4) & 0x3
    }

    /// Pull the next HCI packet from the controller into `buf`, completing
    /// a fragmented ACL packet if necessary. Leaves `buf` empty (0 bytes)
    /// when the controller has nothing queued.
    fn fill(&mut self) -> Result<(), <Self as embedded_io_06::ErrorType>::Error> {
        self.start = 0;
        self.end = self.inner.next(&mut self.buf)?;
        let n = self.end;
        if n < ACL_HDR + 4 || self.buf[0] != HCI_ACL || Self::acl_pb_flag(&self.buf) == 0b01 {
            return Ok(());
        }
        let acl_len = u16::from_le_bytes([self.buf[3], self.buf[4]]) as usize;
        let want = u16::from_le_bytes([self.buf[5], self.buf[6]]) as usize + 4;
        if acl_len >= want {
            return Ok(());
        }

        // First fragment of a longer L2CAP PDU: gather the rest.
        let deadline = millis() + 200;
        let mut tmp = [0u8; 512];
        let mut stray = [0u8; 512];
        let mut stray_len = 0usize;
        while self.end - ACL_HDR < want {
            let m = self.inner.next(&mut tmp)?;
            if m == 0 {
                if millis() > deadline {
                    warn!("HCI: ACL continuation timeout ({}/{} B)", self.end - ACL_HDR, want);
                    break;
                }
                continue;
            }
            if m >= ACL_HDR && tmp[0] == HCI_ACL && Self::acl_pb_flag(&tmp) == 0b01 {
                let payload = &tmp[ACL_HDR..m];
                let k = payload.len().min(self.buf.len() - self.end);
                self.buf[self.end..self.end + k].copy_from_slice(&payload[..k]);
                self.end += k;
            } else if stray_len + m <= stray.len() {
                // e.g. an HCI event: keep it for after the merged packet.
                stray[stray_len..stray_len + m].copy_from_slice(&tmp[..m]);
                stray_len += m;
            } else {
                warn!("HCI: dropping {m}-byte packet during ACL reassembly");
            }
        }
        let new_len = (self.end - ACL_HDR) as u16;
        self.buf[3..5].copy_from_slice(&new_len.to_le_bytes());
        let k = stray_len.min(self.buf.len() - self.end);
        self.buf[self.end..self.end + k].copy_from_slice(&stray[..k]);
        self.end += k;
        Ok(())
    }
}

impl embedded_io_06::ErrorType for BufferedHci {
    type Error = esp_radio::ble::controller::BleConnectorError;
}

impl embedded_io_06::Read for BufferedHci {
    fn read(&mut self, out: &mut [u8]) -> Result<usize, Self::Error> {
        if self.start == self.end {
            self.fill()?;
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
const DEFAULT_ACK_EVERY: u8 = 8;

/// 128-bit UUID `50415045-5250-4c41-5445-0000000000xx` ("PAPERPLATE"),
/// in the little-endian byte order the ATT layer uses.
const fn uuid_le(last: u8) -> [u8; 16] {
    [
        last, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45, 0x54, 0x41, 0x4c, 0x50, 0x52, 0x45, 0x50, 0x41,
        0x50,
    ]
}
const UUID_SVC: [u8; 16] = uuid_le(0x01);
const UUID_CTRL: [u8; 16] = uuid_le(0x02);
const UUID_DATA: [u8; 16] = uuid_le(0x03);
const UUID_STATUS: [u8; 16] = uuid_le(0x04);
const UUID_CONTENT: [u8; 16] = uuid_le(0x05);

// Attribute handles (1-based, in table order).
const H_CTRL_VAL: u16 = 3;
const H_DATA_VAL: u16 = 5;
const H_STATUS_VAL: u16 = 7;
const H_CONTENT_VAL: u16 = 10;
/// Plate text payload cap (the NTAG user area is 888 B anyway).
const MAX_CONTENT_BYTES: usize = 1024;

const PROP_READ: u8 = 0x02;
const PROP_WRITE_NO_RSP: u8 = 0x04;
const PROP_WRITE: u8 = 0x08;
const PROP_NOTIFY: u8 = 0x10;

/// Characteristic declaration value: `props | value_handle:u16 | uuid128`.
const fn char_decl(props: u8, value_handle: u16, uuid: [u8; 16]) -> [u8; 19] {
    let mut d = [0u8; 19];
    d[0] = props;
    d[1] = value_handle as u8;
    d[2] = (value_handle >> 8) as u8;
    let mut i = 0;
    while i < 16 {
        d[3 + i] = uuid[i];
        i += 1;
    }
    d
}

pub fn millis() -> u64 {
    esp_hal::time::Instant::now().duration_since_epoch().as_millis()
}

fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xEDB8_8320 } else { crc >> 1 };
        }
    }
    !crc
}

const ST_IDLE: u8 = 0;
const ST_RECEIVING: u8 = 1;
const ST_COMMITTED: u8 = 2;
const ST_CRC_ERROR: u8 = 3;
const ST_INCOMPLETE: u8 = 4;

const KIND_IMAGE: u8 = 1;
const KIND_CONTENT: u8 = 2;

#[derive(Default)]
pub struct ImgRx {
    state: u8,
    /// What the current transfer carries (`KIND_*`).
    kind: u8,
    width: u16,
    height: u16,
    expected: usize,
    crc: u32,
    ack_every: u8,
    data: Vec<u8>,
    done: Option<MonoImage>,
    done_content: Option<Vec<u8>>,
    /// millis() of the last GATT interaction (for BLE-priority scheduling).
    last_ms: u64,
    /// Client subscribed to STATUS notifications (CCCD).
    notify_on: bool,
    /// A STATUS notification should be sent at the next opportunity.
    pending_ntf: bool,
    /// Accepted packets since the last acknowledgement.
    since_ack: u8,
    /// A NAK was already sent for the current out-of-order streak.
    nak_sent: bool,
    /// Bench: CTRL `05` requests a short doze test (light sleep a few
    /// times, then come back and report).
    doze_test_req: bool,
}

impl ImgRx {
    fn receiving(&self) -> bool {
        self.state == ST_RECEIVING
    }

    fn ctrl(&mut self, d: &[u8]) {
        self.last_ms = millis();
        match d.first() {
            Some(0x11) if d.len() >= 10 => {
                let len = u32::from_le_bytes([d[1], d[2], d[3], d[4]]) as usize;
                let crc = u32::from_le_bytes([d[5], d[6], d[7], d[8]]);
                let ack_every = if d[9] == 0 { DEFAULT_ACK_EVERY } else { d[9] };
                if len == 0 || len > MAX_CONTENT_BYTES {
                    warn!("BLE content: bad start len={len}");
                    self.reset();
                    self.pending_ntf = true;
                    return;
                }
                info!("BLE content: start ({len} B, ack every {ack_every})");
                self.reset();
                self.state = ST_RECEIVING;
                self.kind = KIND_CONTENT;
                self.expected = len;
                self.crc = crc;
                self.ack_every = ack_every;
                self.data = Vec::with_capacity(len);
                self.pending_ntf = true;
            }
            Some(0x10) if d.len() >= 14 => {
                let w = u16::from_le_bytes([d[1], d[2]]);
                let h = u16::from_le_bytes([d[3], d[4]]);
                let len = u32::from_le_bytes([d[5], d[6], d[7], d[8]]) as usize;
                let crc = u32::from_le_bytes([d[9], d[10], d[11], d[12]]);
                let ack_every = if d[13] == 0 { DEFAULT_ACK_EVERY } else { d[13] };
                let row_bytes = (w as usize).div_ceil(8);
                if w == 0 || h == 0 || w > 480 || h > 800 || len != row_bytes * h as usize || len > MAX_IMAGE_BYTES {
                    warn!("BLE img: bad start {w}x{h} len={len}");
                    self.reset();
                    self.pending_ntf = true;
                    return;
                }
                info!("BLE img: start {w}x{h} ({len} B, ack every {ack_every})");
                self.reset();
                self.state = ST_RECEIVING;
                self.kind = KIND_IMAGE;
                self.width = w;
                self.height = h;
                self.expected = len;
                self.crc = crc;
                self.ack_every = ack_every;
                self.data = Vec::with_capacity(len);
                self.pending_ntf = true;
            }
            Some(0x02) => {
                if !self.receiving() {
                    warn!("BLE img: commit while idle");
                } else if self.data.len() != self.expected {
                    warn!("BLE img: commit with {}/{} B", self.data.len(), self.expected);
                    self.state = ST_INCOMPLETE;
                    self.data = Vec::new();
                } else if crc32(&self.data) != self.crc {
                    warn!("BLE img: CRC mismatch");
                    self.state = ST_CRC_ERROR;
                    self.data = Vec::new();
                } else if self.kind == KIND_CONTENT {
                    info!("BLE content: commit ({} B, CRC ok)", self.data.len());
                    self.state = ST_COMMITTED;
                    self.done_content = Some(core::mem::take(&mut self.data));
                } else {
                    info!("BLE img: commit ({} B, CRC ok)", self.data.len());
                    self.state = ST_COMMITTED;
                    self.done = Some(MonoImage {
                        width: self.width,
                        height: self.height,
                        bits: core::mem::take(&mut self.data),
                    });
                }
                self.pending_ntf = true;
            }
            Some(0x03) => {
                info!("BLE img: abort");
                self.reset();
                self.pending_ntf = true;
            }
            Some(0x04) => {
                self.pending_ntf = true;
            }
            Some(0x05) => {
                info!("BLE: doze test requested");
                self.doze_test_req = true;
            }
            _ => warn!("BLE img: unknown ctrl {d:02X?}"),
        }
    }

    fn push(&mut self, d: &[u8]) {
        self.last_ms = millis();
        if !self.receiving() || d.len() < 3 {
            return;
        }
        let off = u16::from_le_bytes([d[0], d[1]]) as usize;
        let payload = &d[2..];
        if off == self.data.len() && self.data.len() + payload.len() <= self.expected {
            self.data.extend_from_slice(payload);
            self.nak_sent = false;
            self.since_ack += 1;
            if self.since_ack >= self.ack_every || self.data.len() == self.expected {
                self.since_ack = 0;
                self.pending_ntf = true;
            }
        } else if !self.nak_sent {
            warn!("BLE img: NAK off={off} have={} +{} exp={}", self.data.len(), payload.len(), self.expected);
            self.nak_sent = true;
            self.since_ack = 0;
            self.pending_ntf = true;
        }
    }

    fn status_bytes(&self) -> [u8; 9] {
        let mut out = [0u8; 9];
        out[0] = self.state;
        out[1..5].copy_from_slice(&(self.data.len() as u32).to_le_bytes());
        out[5..9].copy_from_slice(&(self.expected as u32).to_le_bytes());
        out
    }

    fn status(&mut self, out: &mut [u8]) -> usize {
        self.last_ms = millis();
        if out.len() < 9 {
            return 0;
        }
        out[..9].copy_from_slice(&self.status_bytes());
        9
    }

    /// Take the pending STATUS notification, if the client subscribed.
    fn take_notification(&mut self) -> Option<[u8; 9]> {
        if !self.pending_ntf {
            return None;
        }
        self.pending_ntf = false;
        self.notify_on.then(|| self.status_bytes())
    }

    fn reset(&mut self) {
        let notify_on = self.notify_on;
        *self = Self::default();
        self.notify_on = notify_on;
        self.last_ms = millis();
    }
}

/// Why a BLE session ended.
pub(crate) enum SessionEnd {
    /// Disconnect or stack error: start a new session right away.
    Restart,
    /// Nothing happened for a long time: advertising is off, the caller
    /// should doze (light sleep) until something wakes the plate.
    Doze,
}

impl super::App {
    /// One BLE session: advertise, serve GATT until disconnect (or error),
    /// running the normal app tick in between. Returns to be called again.
    pub(crate) fn ble_session(&mut self, hci: &HciConnector<BufferedHci>) -> SessionEnd {
        let mut ble = Ble::new(hci);
        if let Err(e) = ble.init() {
            error!("BLE stack init failed: {e:?}");
            self.board.delay.delay_ms(1000);
            return SessionEnd::Restart;
        }
        // 200-250 ms advertising interval (bleps' default is 160 ms): still
        // found within a second by phones, noticeably less radio time.
        // bleps (rev a5148d8) serialises the interval fields big-endian while
        // HCI is little-endian, so pre-swap the bytes; without this the
        // controller rejects the command (status 0x12).
        let adv = AdvertisingParameters {
            advertising_interval_min: 0x0140u16.swap_bytes(),
            advertising_interval_max: 0x0190u16.swap_bytes(),
            advertising_type: AdvertisingType::AdvInd,
            own_address_type: OwnAddressType::Public,
            peer_address_type: PeerAddressType::Public,
            peer_address: [0; 6],
            advertising_channel_map: 0x07,
            filter_policy: AdvertisingFilterPolicy::All,
        };
        if let Err(e) = ble.cmd_set_le_advertising_parameters_custom(&adv) {
            warn!("BLE adv params failed ({e:?}); using defaults");
            let _ = ble.cmd_set_le_advertising_parameters();
        }
        match create_advertising_data(&[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(DEVICE_NAME),
        ]) {
            Ok(data) => {
                let _ = ble.cmd_set_le_advertising_data(data);
            }
            Err(e) => {
                error!("BLE adv data failed: {e:?}");
                return SessionEnd::Restart;
            }
        }
        let _ = ble.cmd_set_le_advertise_enable(true);
        log::debug!("BLE advertising as {DEVICE_NAME}");

        let rx = RefCell::new(ImgRx::default());
        let cccd = RefCell::new([0u8; 2]);

        // --- attribute table -------------------------------------------------
        // 1: primary service
        let mut svc_val: &[u8; 16] = &UUID_SVC;
        // 2/3: CTRL declaration + value
        let ctrl_decl = char_decl(PROP_WRITE | PROP_WRITE_NO_RSP, H_CTRL_VAL, UUID_CTRL);
        let mut ctrl_decl_val: &[u8; 19] = &ctrl_decl;
        let mut wf_ctrl = |_offset: usize, data: &[u8]| rx.borrow_mut().ctrl(data);
        let mut ctrl_val = ((), &mut wf_ctrl, ());
        // 4/5: DATA declaration + value
        let data_decl = char_decl(PROP_WRITE | PROP_WRITE_NO_RSP, H_DATA_VAL, UUID_DATA);
        let mut data_decl_val: &[u8; 19] = &data_decl;
        let mut wf_data = |_offset: usize, data: &[u8]| rx.borrow_mut().push(data);
        let mut data_val = ((), &mut wf_data, ());
        // 6/7: STATUS declaration + value, 8: CCCD
        let status_decl = char_decl(PROP_READ | PROP_NOTIFY, H_STATUS_VAL, UUID_STATUS);
        let mut status_decl_val: &[u8; 19] = &status_decl;
        let mut rf_status = |_offset: usize, out: &mut [u8]| -> usize { rx.borrow_mut().status(out) };
        let mut nf_status = |enabled: bool| {
            info!("BLE: STATUS notifications {}", if enabled { "on" } else { "off" });
            rx.borrow_mut().notify_on = enabled;
        };
        let mut status_val = (&mut rf_status, (), &mut nf_status);
        let mut rf_cccd = |offset: usize, out: &mut [u8]| -> usize {
            let c = cccd.borrow();
            if offset >= 2 {
                return 0;
            }
            let n = (2 - offset).min(out.len());
            out[..n].copy_from_slice(&c[offset..offset + n]);
            n
        };
        let mut wf_cccd = |offset: usize, d: &[u8]| {
            let mut c = cccd.borrow_mut();
            if offset < 2 {
                let n = (2 - offset).min(d.len());
                c[offset..offset + n].copy_from_slice(&d[..n]);
            }
        };
        let mut cccd_val = (&mut rf_cccd, &mut wf_cccd, ());
        // 9/10: CONTENT declaration + value (current plate text, long-read
        // capable via the offset).
        let content_text = RefCell::new(self.content.to_plain().into_bytes());
        let content_decl = char_decl(PROP_READ, H_CONTENT_VAL, UUID_CONTENT);
        let mut content_decl_val: &[u8; 19] = &content_decl;
        let mut rf_content = |offset: usize, out: &mut [u8]| -> usize {
            let t = content_text.borrow();
            if offset >= t.len() {
                return 0;
            }
            let n = (t.len() - offset).min(out.len());
            out[..n].copy_from_slice(&t[offset..offset + n]);
            n
        };
        let mut content_val = (&mut rf_content, (), ());

        let mut gatt_attributes = [
            Attribute::new(PRIMARY_SERVICE_UUID16, &mut svc_val),
            Attribute::new(CHARACTERISTIC_UUID16, &mut ctrl_decl_val),
            Attribute::new(Uuid::Uuid128(UUID_CTRL), &mut ctrl_val),
            Attribute::new(CHARACTERISTIC_UUID16, &mut data_decl_val),
            Attribute::new(Uuid::Uuid128(UUID_DATA), &mut data_val),
            Attribute::new(CHARACTERISTIC_UUID16, &mut status_decl_val),
            Attribute::new(Uuid::Uuid128(UUID_STATUS), &mut status_val),
            Attribute::new(Uuid::Uuid16(0x2902), &mut cccd_val),
            Attribute::new(CHARACTERISTIC_UUID16, &mut content_decl_val),
            Attribute::new(Uuid::Uuid128(UUID_CONTENT), &mut content_val),
        ];

        let mut rng = NoRng;
        let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes, &mut rng);

        let mut errors = 0u32;
        // Last time the controller handed us anything (connection traffic
        // counts as activity even without GATT operations).
        let mut hci_last_ms = millis();
        let end = 'session: loop {
            // Pump HCI while the controller has something for us (or we have
            // a notification to send). Advertising runs in the controller on
            // its own, so with nothing queued there is nothing to do and the
            // main task can sleep.
            for _ in 0..96 {
                let ntf = rx
                    .borrow_mut()
                    .take_notification()
                    .map(|b| NotificationData::new(H_STATUS_VAL, &b));
                if ntf.is_none() && !esp_radio::ble::have_hci_read_data() {
                    break;
                }
                hci_last_ms = millis();
                match srv.do_work_with_notification(ntf) {
                    Ok(WorkResult::GotDisconnected) => {
                        info!("BLE: disconnected");
                        break 'session SessionEnd::Restart;
                    }
                    Ok(WorkResult::DidWork) => {}
                    Err(e) => {
                        errors += 1;
                        if errors >= 8 {
                            warn!("BLE: {errors} work errors ({e:?}); restarting session");
                            break 'session SessionEnd::Restart;
                        }
                        break;
                    }
                }
            }

            // Apply a committed image only after its STATUS notification went
            // out (the e-paper refresh blocks the HCI pump for seconds).
            let img = {
                let mut r = rx.borrow_mut();
                if r.pending_ntf {
                    None
                } else {
                    r.done.take()
                }
            };
            if let Some(img) = img {
                self.on_ble_image(img);
                rx.borrow_mut().state = ST_IDLE;
                errors = 0;
            }
            let text = {
                let mut r = rx.borrow_mut();
                if r.pending_ntf {
                    None
                } else {
                    r.done_content.take()
                }
            };
            if let Some(text) = text {
                self.on_ble_content(&text);
                *content_text.borrow_mut() = self.content.to_plain().into_bytes();
                rx.borrow_mut().state = ST_IDLE;
                errors = 0;
            }

            // While a transfer is actively streaming, do NOTHING but pump HCI:
            // even a 1-2 ms touch I2C read here lets the controller's RX queue
            // overflow under the phone's write-without-response burst.
            if rx.borrow().receiving() {
                continue;
            }

            // Between transfers but with a central recently active: cheap
            // housekeeping only, keep the link responsive.
            let ble_active = millis().saturating_sub(rx.borrow().last_ms) < 3000;
            if ble_active {
                self.tick_light();
            } else {
                if rx.borrow().doze_test_req {
                    self.doze_test = true;
                    break 'session SessionEnd::Doze;
                }
                if millis().saturating_sub(hci_last_ms) > super::DOZE_AFTER_MS && self.should_doze() {
                    break 'session SessionEnd::Doze;
                }
                // Keep the BLE service cadence tight so advertising stays
                // continuous (discoverable) while NFC emulation still runs.
                self.tick(true, 6);
            }
        };

        // The bench client disconnects right after asking: honour the
        // request even when the session ended with the disconnect.
        let end = if rx.borrow().doze_test_req {
            self.doze_test = true;
            SessionEnd::Doze
        } else {
            end
        };

        if matches!(end, SessionEnd::Doze) {
            // The link layer runs on this CPU: it cannot advertise while we
            // light-sleep, so stop it cleanly. The next session re-inits the
            // stack and starts advertising again.
            // `srv` holds `ble` for its whole lifetime; a fresh handle on
            // the same HCI link is enough to send one command.
            let mut ble = Ble::new(hci);
            match ble.cmd_set_le_advertise_enable(false) {
                Ok(_) => {
                    super::doze_diag_set(4, 1);
                    info!("BLE: advertising off (doze)");
                }
                Err(e) => {
                    super::doze_diag_set(4, 2);
                    warn!("BLE: advertising off failed: {e:?}");
                }
            }
        }
        end
    }

    /// Cheap housekeeping used while a BLE central is active: touch,
    /// buttons and the sleep/heartbeat bookkeeping, but no NFC emulation.
    fn tick_light(&mut self) {
        self.handle_touch();
        self.handle_buttons();
        self.manage_epd_sleep();
        super::idle_sleep_ms(2);
    }
}
