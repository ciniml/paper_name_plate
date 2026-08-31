//! NDEF Type 4 Tag reading over ISO-DEP, plus a minimal NDEF record parser.

use alloc::string::String;
use alloc::vec::Vec;
use core::fmt::Write as _;

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::isodep::{Error as DepError, IsoDep};
use crate::st25r3916::St25r3916;

/// NDEF Type 4 application AID.
const NDEF_AID: [u8; 7] = [0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01];

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error<E> {
    Dep(DepError<E>),
    /// APDU answered with a non-9000 status word.
    Status { step: &'static str, sw: u16 },
    Malformed,
}

impl<E> From<DepError<E>> for Error<E> {
    fn from(e: DepError<E>) -> Self {
        Error::Dep(e)
    }
}

fn check<E>(step: &'static str, rsp: &[u8]) -> Result<(), Error<E>> {
    if rsp.len() < 2 {
        return Err(Error::Malformed);
    }
    let sw = u16::from_be_bytes([rsp[rsp.len() - 2], rsp[rsp.len() - 1]]);
    if sw != 0x9000 {
        return Err(Error::Status { step, sw });
    }
    Ok(())
}

/// Read the raw NDEF message from a Type 4 tag (already ISO-DEP activated).
pub fn read_type4<I: I2c>(
    dep: &mut IsoDep,
    nfc: &mut St25r3916<I>,
    delay: &mut impl DelayNs,
) -> Result<Vec<u8>, Error<I::Error>> {
    let mut rsp: Vec<u8> = Vec::new();

    // SELECT NDEF application.
    let mut apdu: Vec<u8> = Vec::new();
    apdu.extend_from_slice(&[0x00, 0xA4, 0x04, 0x00, NDEF_AID.len() as u8]);
    apdu.extend_from_slice(&NDEF_AID);
    apdu.push(0x00);
    dep.exchange(nfc, delay, &apdu, &mut rsp)?;
    check("select-app", &rsp)?;

    // SELECT the capability container (E103) and read it.
    dep.exchange(nfc, delay, &[0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x03], &mut rsp)?;
    check("select-cc", &rsp)?;
    dep.exchange(nfc, delay, &[0x00, 0xB0, 0x00, 0x00, 0x0F], &mut rsp)?;
    check("read-cc", &rsp)?;
    if rsp.len() < 2 + 15 {
        return Err(Error::Malformed);
    }
    let cc = &rsp[..15];
    // CC: len(2) ver(1) MLe(2) MLc(2) then NDEF file control TLV (T=04 L=06:
    // file id(2) max size(2) read access(1) write access(1)).
    let mle = u16::from_be_bytes([cc[3], cc[4]]).clamp(15, 246) as usize;
    if cc[7] != 0x04 {
        return Err(Error::Malformed);
    }
    let file_id = [cc[9], cc[10]];

    // SELECT the NDEF file and read NLEN.
    dep.exchange(nfc, delay, &[0x00, 0xA4, 0x00, 0x0C, 0x02, file_id[0], file_id[1]], &mut rsp)?;
    check("select-ndef", &rsp)?;
    dep.exchange(nfc, delay, &[0x00, 0xB0, 0x00, 0x00, 0x02], &mut rsp)?;
    check("read-nlen", &rsp)?;
    if rsp.len() < 4 {
        return Err(Error::Malformed);
    }
    let nlen = u16::from_be_bytes([rsp[0], rsp[1]]) as usize;

    // Read the message in MLe-sized chunks starting at offset 2.
    let mut msg: Vec<u8> = Vec::with_capacity(nlen);
    let mut offset = 2usize;
    while msg.len() < nlen {
        let chunk = (nlen - msg.len()).min(mle).min(255);
        let off = (offset as u16).to_be_bytes();
        dep.exchange(nfc, delay, &[0x00, 0xB0, off[0], off[1], chunk as u8], &mut rsp)?;
        check("read-ndef", &rsp)?;
        if rsp.len() < 2 + chunk {
            return Err(Error::Malformed);
        }
        msg.extend_from_slice(&rsp[..chunk]);
        offset += chunk;
    }
    Ok(msg)
}

/// A human-readable summary of the first record of an NDEF message.
pub fn summarize(msg: &[u8]) -> String {
    let mut out = String::new();
    match first_record(msg) {
        Some((tnf, rtype, payload)) => match (tnf, rtype) {
            (1, b"U") => {
                let _ = write!(out, "URI: {}{}", uri_prefix(payload.first().copied().unwrap_or(0)), ascii(&payload[1.min(payload.len())..]));
            }
            (1, b"T") => {
                let lang_len = (payload.first().copied().unwrap_or(0) & 0x3F) as usize;
                let text = payload.get(1 + lang_len..).unwrap_or(&[]);
                let _ = write!(out, "Text: {}", ascii(text));
            }
            _ => {
                let _ = write!(out, "TNF={tnf} type={} ({}B)", ascii(rtype), payload.len());
            }
        },
        None => out.push_str("(empty/invalid NDEF)"),
    }
    out
}

/// Parse the first record: returns (TNF, type, payload).
fn first_record(msg: &[u8]) -> Option<(u8, &[u8], &[u8])> {
    if msg.is_empty() {
        return None;
    }
    let hdr = msg[0];
    let tnf = hdr & 0x07;
    let sr = hdr & 0x10 != 0;
    let il = hdr & 0x08 != 0;
    let mut i = 1usize;
    let type_len = *msg.get(i)? as usize;
    i += 1;
    let payload_len = if sr {
        let l = *msg.get(i)? as usize;
        i += 1;
        l
    } else {
        let b = msg.get(i..i + 4)?;
        i += 4;
        u32::from_be_bytes([b[0], b[1], b[2], b[3]]) as usize
    };
    if il {
        let idl = *msg.get(i)? as usize;
        i += 1 + idl;
    }
    let rtype = msg.get(i..i + type_len)?;
    i += type_len;
    let payload = msg.get(i..i + payload_len)?;
    Some((tnf, rtype, payload))
}

fn ascii(bytes: &[u8]) -> String {
    bytes.iter().map(|&b| if (0x20..0x7F).contains(&b) { b as char } else { '.' }).collect()
}

fn uri_prefix(code: u8) -> &'static str {
    match code {
        0x01 => "http://www.",
        0x02 => "https://www.",
        0x03 => "http://",
        0x04 => "https://",
        0x05 => "tel:",
        0x06 => "mailto:",
        _ => "",
    }
}
