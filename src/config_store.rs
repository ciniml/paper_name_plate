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
/// Raw flash area past the factory partition (0x10000 + 0xFA0000): 320 KiB
/// of unallocated flash used for the (large) display image.
const IMAGE_OFFSET: u32 = 0xFB_0000;
const IMAGE_REGION: usize = 0x50000;
const MAGIC: [u8; 4] = *b"PNP1";
const HEADER: usize = 8;
/// Keep well below the region (and the NTAG user area is 888 B anyway).
pub const MAX_DATA: usize = 2048;
/// Image payload cap: 4-byte header + 480x800/8 bits.
pub const MAX_IMAGE: usize = 48_004;

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

fn load_rec(flash: &mut FlashStorage<'_>, offset: u32, max: usize, region: usize) -> Option<Vec<u8>> {
    let mut hdr = [0u8; HEADER];
    flash.read(offset, &mut hdr).ok()?;
    if hdr[0..4] != MAGIC {
        return None;
    }
    let len = u16::from_le_bytes([hdr[4], hdr[5]]) as usize
        | ((hdr[6] as usize & 0x0F) << 16); // 20-bit length (crc uses 12 bits? no: see save)
    let crc = hdr[7];
    if len == 0 || len > max || HEADER + len > region {
        return None;
    }
    let mut data = alloc::vec![0u8; len];
    flash.read(offset + HEADER as u32, &mut data).ok()?;
    ((crc16(&data) & 0xFF) as u8 == crc).then_some(data)
}

fn save_rec(
    flash: &mut FlashStorage<'_>,
    offset: u32,
    max: usize,
    data: &[u8],
) -> Result<(), esp_storage::FlashStorageError> {
    let len = data.len().min(max);
    let data = &data[..len];
    let mut buf: Vec<u8> = Vec::with_capacity(HEADER + len);
    buf.extend_from_slice(&MAGIC);
    buf.extend_from_slice(&(len as u16).to_le_bytes());
    buf.push((len >> 16) as u8 & 0x0F);
    buf.push((crc16(data) & 0xFF) as u8);
    buf.extend_from_slice(data);
    flash.write(offset, &buf)
}

/// Load the stored content record, if a valid one is present.
pub fn load(flash: &mut FlashStorage<'_>) -> Option<Vec<u8>> {
    load_rec(flash, REGION_OFFSET, MAX_DATA, REGION_SIZE)
}

/// Store the content record, replacing any previous one.
pub fn save(flash: &mut FlashStorage<'_>, data: &[u8]) -> Result<(), esp_storage::FlashStorageError> {
    save_rec(flash, REGION_OFFSET, MAX_DATA, data)
}

/// Load the stored display image (MonoImage::encode payload).
pub fn load_image(flash: &mut FlashStorage<'_>) -> Option<Vec<u8>> {
    load_rec(flash, IMAGE_OFFSET, MAX_IMAGE, IMAGE_REGION)
}

/// Store the display image.
pub fn save_image(flash: &mut FlashStorage<'_>, data: &[u8]) -> Result<(), esp_storage::FlashStorageError> {
    save_rec(flash, IMAGE_OFFSET, MAX_IMAGE, data)
}
