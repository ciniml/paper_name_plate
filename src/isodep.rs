//! Minimal ISO14443-4 (ISO-DEP) initiator on top of [`St25r3916`].
//!
//! Supports: RATS/ATS, I-block exchange without CID/NAD, S(WTX) handling,
//! receive-side chaining, S(DESELECT). Bit rate stays at 106 kbps (no PPS).

use alloc::vec::Vec;

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::st25r3916::{Error as NfcError, St25r3916};

/// Answer To Select.
#[derive(Debug, Clone, Default)]
pub struct Ats {
    pub raw: Vec<u8>,
    /// Frame waiting time derived from TB(1) (milliseconds, clamped).
    pub fwt_ms: u32,
    /// Max frame size the card accepts (FSC).
    pub fsc: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    Nfc(NfcError<E>),
    /// Malformed ATS / protocol violation.
    Protocol,
    /// Too many retries / WTX loops.
    Retries,
}

impl<E> From<NfcError<E>> for Error<E> {
    fn from(e: NfcError<E>) -> Self {
        Error::Nfc(e)
    }
}

const FSCI_TABLE: [usize; 9] = [16, 24, 32, 40, 48, 64, 96, 128, 256];

pub struct IsoDep {
    block_num: bool,
    ats: Ats,
}

impl IsoDep {
    /// Send RATS (FSD = 256, CID = 0) to a tag that was just SELECTed with
    /// SAK bit 0x20 set, and parse the ATS.
    pub fn activate<I: I2c>(
        nfc: &mut St25r3916<I>,
        delay: &mut impl DelayNs,
    ) -> Result<Self, Error<I::Error>> {
        let mut buf = [0u8; 32];
        let n = nfc.transceive(delay, &[0xE0, 0x80], &mut buf, 70)?;
        if n < 3 {
            return Err(Error::Protocol);
        }
        let data = &buf[..n - 2]; // strip CRC
        let tl = data[0] as usize;
        if tl < 1 || tl > data.len() {
            return Err(Error::Protocol);
        }
        let mut ats = Ats { raw: Vec::from(&data[..tl]), fwt_ms: 40, fsc: 32 };
        if tl >= 2 {
            let t0 = data[1];
            ats.fsc = FSCI_TABLE[((t0 & 0x0F) as usize).min(8)];
            let mut idx = 2;
            if t0 & 0x10 != 0 {
                idx += 1; // TA(1): bit rates, ignored
            }
            if t0 & 0x20 != 0 && idx < tl {
                let fwi = (data[idx] >> 4).min(14) as u32;
                // FWT = 256 * 16 / fc * 2^FWI ≈ 0.302 ms * 2^FWI
                ats.fwt_ms = ((302u64 << fwi) / 1000).clamp(5, 3000) as u32;
            }
        }
        Ok(Self { block_num: false, ats })
    }

    pub fn ats(&self) -> &Ats {
        &self.ats
    }

    /// Exchange one APDU: sends `apdu` as (possibly single) I-block and
    /// collects the full response (R-APDU incl. SW1SW2) into `rsp`.
    /// Transmit-side chaining is not implemented; `apdu` must fit in FSC-3.
    pub fn exchange<I: I2c>(
        &mut self,
        nfc: &mut St25r3916<I>,
        delay: &mut impl DelayNs,
        apdu: &[u8],
        rsp: &mut Vec<u8>,
    ) -> Result<(), Error<I::Error>> {
        rsp.clear();
        if apdu.len() + 3 > self.ats.fsc {
            return Err(Error::Protocol);
        }

        let mut tx: Vec<u8> = Vec::with_capacity(apdu.len() + 1);
        tx.push(0x02 | self.block_num as u8); // I-block, no chaining
        tx.extend_from_slice(apdu);

        let mut rx = [0u8; 260];
        let mut timeout = self.ats.fwt_ms;
        let mut guard = 0u32;
        loop {
            guard += 1;
            if guard > 32 {
                return Err(Error::Retries);
            }
            let n = nfc.transceive(delay, &tx, &mut rx, timeout)?;
            if n < 3 {
                return Err(Error::Protocol);
            }
            let pcb = rx[0];
            let payload = &rx[1..n - 2]; // strip CRC

            if pcb & 0xF7 == 0xF2 {
                // S(WTX): grant the requested extension.
                let wtxm = payload.first().copied().unwrap_or(1) & 0x3F;
                timeout = (self.ats.fwt_ms.saturating_mul(wtxm.max(1) as u32)).min(5_000);
                tx.clear();
                tx.push(0xF2);
                tx.push(wtxm);
                continue;
            }
            if pcb & 0xE2 == 0x02 {
                // I-block response.
                if pcb & 0x01 != self.block_num as u8 {
                    // Unexpected block number; treat as protocol error.
                    return Err(Error::Protocol);
                }
                rsp.extend_from_slice(payload);
                self.block_num = !self.block_num;
                if pcb & 0x10 != 0 {
                    // Card is chaining: acknowledge with R(ACK) of the new number.
                    timeout = self.ats.fwt_ms;
                    tx.clear();
                    tx.push(0xA2 | self.block_num as u8);
                    continue;
                }
                return Ok(());
            }
            if pcb & 0xF6 == 0xA2 {
                // R-block from the card (NAK/ACK) — not expected here.
                return Err(Error::Protocol);
            }
            return Err(Error::Protocol);
        }
    }

    /// S(DESELECT): cleanly ends the ISO-DEP session (tag goes to HALT).
    pub fn deselect<I: I2c>(
        &mut self,
        nfc: &mut St25r3916<I>,
        delay: &mut impl DelayNs,
    ) -> Result<(), Error<I::Error>> {
        let mut rx = [0u8; 8];
        let n = nfc.transceive(delay, &[0xC2], &mut rx, self.ats.fwt_ms)?;
        if n >= 3 && rx[0] == 0xC2 {
            Ok(())
        } else {
            Err(Error::Protocol)
        }
    }
}
