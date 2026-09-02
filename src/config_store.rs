//! Persistent storage for the plate's NDEF content.
//!
//! Uses the (otherwise unused) `nvs` partition of the stock partition table
//! as a raw record area: flash offset 0x9000, 24 KiB. One record lives at
//! the start of the region:
//!
//! `magic "PNP1" (4) | len u16 LE (2) | crc16 u16 LE (2) | data (len)`
//!
//! CRC: CCITT-FALSE over `data`. `esp-storage`'s [`FlashStorage`] handles the
//! erase/read-modify-write cycle behind the [`Storage`] trait.

use alloc::vec::Vec;

use embedded_storage::{ReadStorage, Storage};
use esp_storage::FlashStorage;

const REGION_OFFSET: u32 = 0x9000;
const REGION_SIZE: usize = 0x6000;
const MAGIC: [u8; 4] = *b"PNP1";
const HEADER: usize = 8;
/// Keep well below the region (and the NTAG user area is 888 B anyway).
pub const MAX_DATA: usize = 2048;

fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
        }
    }
    crc
}

/// Load the stored record, if a valid one is present.
pub fn load(flash: &mut FlashStorage<'_>) -> Option<Vec<u8>> {
    let mut hdr = [0u8; HEADER];
    flash.read(REGION_OFFSET, &mut hdr).ok()?;
    if hdr[0..4] != MAGIC {
        return None;
    }
    let len = u16::from_le_bytes([hdr[4], hdr[5]]) as usize;
    let crc = u16::from_le_bytes([hdr[6], hdr[7]]);
    if len == 0 || len > MAX_DATA || HEADER + len > REGION_SIZE {
        return None;
    }
    let mut data = alloc::vec![0u8; len];
    flash.read(REGION_OFFSET + HEADER as u32, &mut data).ok()?;
    (crc16(&data) == crc).then_some(data)
}

/// Store `data`, replacing any previous record.
pub fn save(flash: &mut FlashStorage<'_>, data: &[u8]) -> Result<(), esp_storage::FlashStorageError> {
    let len = data.len().min(MAX_DATA);
    let data = &data[..len];
    let mut buf: Vec<u8> = Vec::with_capacity(HEADER + len);
    buf.extend_from_slice(&MAGIC);
    buf.extend_from_slice(&(len as u16).to_le_bytes());
    buf.extend_from_slice(&crc16(data).to_le_bytes());
    buf.extend_from_slice(data);
    flash.write(REGION_OFFSET, &buf)
}
