//! NFC Forum Type 2 Tag emulation (NTAG216-like) on the ST25R3916.
//!
//! The chip's passive-target logic answers REQA/WUPA, anticollision and
//! SELECT by itself (PT memory holds UID/ATQA/SAK). Once activated we serve
//! the Type 2 command set: READ, FAST_READ, WRITE, GET_VERSION, READ_SIG,
//! HLTA. Everything else gets a NAK.
//!
//! Memory layout (4-byte pages): 0-1 UID/BCC, 2 lock, 3 capability
//! container `E1 10 6D 00` (NDEF, v1.0, 888 B user area), 4.. user memory
//! holding the NDEF TLV, then the NTAG216 config pages.
//!
//! The state machine follows M5Unit-NFC's `emulation_layer_a_ST25R3916.cpp`
//! (Off → Idle → Ready → Active → Halt), driven by polling the interrupt
//! registers from [`T2tEmulator::update`].

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::st25r3916::{cmd, irq, reg, St25r3916};

/// NTAG216: 231 pages.
pub const PAGES: usize = 231;
pub const MEM_BYTES: usize = PAGES * 4;
/// First and last user-memory page.
pub const USER_FIRST_PAGE: usize = 4;
pub const USER_LAST_PAGE: usize = 225;
pub const USER_BYTES: usize = (USER_LAST_PAGE - USER_FIRST_PAGE + 1) * 4; // 888

const ATQA: [u8; 2] = [0x44, 0x00];
const SAK_CASCADE: u8 = 0x04;
const GET_VERSION_NTAG216: [u8; 8] = [0x00, 0x04, 0x04, 0x02, 0x01, 0x00, 0x13, 0x03];
const ACK: u8 = 0x0A;
const NAK: u8 = 0x00;

// Operation control / NFCIP-1 passive target definition bits.
const OP_EN: u8 = 0x80;
const OP_RX_EN: u8 = 0x40;
const OP_TX_EN: u8 = 0x08;
const OP_WU: u8 = 0x04;
const D_AC_AP2P: u8 = 0x08;
const D_212_424_1R: u8 = 0x04;
const D_106_AC_A: u8 = 0x01;
const AUX_NO_CRC_RX: u8 = 0x80;
const MODE_TARGET_BITRATE_DETECT: u8 = 0x80 | (0x09 << 3);
const MODE_TARGET_LISTEN_A: u8 = 0x80 | (0x01 << 3);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// No external field; chip in low-power sense.
    Off,
    /// Field present, waiting for the chip's automatic anticollision.
    Idle,
    /// Anticollision in progress.
    Ready,
    /// Selected; serving commands.
    Active,
    /// HLTA received; only WUPA wakes us.
    Halt,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    None,
    /// A reader's field appeared.
    FieldOn,
    /// The field went away; `written` tells whether memory was modified
    /// during the session.
    FieldOff { written: bool },
    /// We were selected by a reader.
    Selected,
}

pub struct T2tEmulator {
    pub memory: [u8; MEM_BYTES],
    state: State,
    bitrate_106: bool,
    wakeup: bool,
    dirty: bool,
    /// Commands served since the last field-on.
    pub commands: u32,
    /// Last command byte seen (diagnostics).
    pub last_cmd: u8,
}

impl T2tEmulator {
    /// Empty tag with the given 7-byte UID.
    pub fn new(uid: [u8; 7]) -> Self {
        let mut memory = [0u8; MEM_BYTES];
        // Pages 0-1: UID with cascade tag + BCCs.
        memory[0] = uid[0];
        memory[1] = uid[1];
        memory[2] = uid[2];
        memory[3] = 0x88 ^ uid[0] ^ uid[1] ^ uid[2];
        memory[4..8].copy_from_slice(&uid[3..7]);
        memory[8] = uid[3] ^ uid[4] ^ uid[5] ^ uid[6];
        // Page 2: internal + lock bytes (all unlocked).
        memory[9] = 0x48;
        // Page 3: capability container.
        memory[12..16].copy_from_slice(&[0xE1, 0x10, 0x6D, 0x00]);
        // Empty NDEF TLV.
        memory[16..20].copy_from_slice(&[0x03, 0x00, 0xFE, 0x00]);
        // NTAG216 config pages (226..230): defaults.
        let cfg = USER_LAST_PAGE + 1;
        memory[cfg * 4..cfg * 4 + 4].copy_from_slice(&[0x00, 0x00, 0x00, 0xFF]); // MIRROR/AUTH0
        memory[cfg * 4 + 4..cfg * 4 + 8].copy_from_slice(&[0x00, 0x05, 0x00, 0x00]); // ACCESS
        memory[cfg * 4 + 8..cfg * 4 + 12].copy_from_slice(&[0xFF; 4]); // PWD
        Self { memory, state: State::Off, bitrate_106: false, wakeup: false, dirty: false, commands: 0, last_cmd: 0 }
    }

    pub fn uid(&self) -> [u8; 7] {
        [self.memory[0], self.memory[1], self.memory[2], self.memory[4], self.memory[5], self.memory[6], self.memory[7]]
    }

    pub fn state(&self) -> State {
        self.state
    }

    /// Store an NDEF message as the (only) NDEF TLV in user memory.
    /// Returns `false` if it does not fit.
    pub fn set_ndef(&mut self, msg: &[u8]) -> bool {
        let hdr = if msg.len() < 0xFF { 2 } else { 4 };
        if hdr + msg.len() + 1 > USER_BYTES {
            return false;
        }
        let base = USER_FIRST_PAGE * 4;
        let mut i = base;
        self.memory[i] = 0x03;
        i += 1;
        if hdr == 2 {
            self.memory[i] = msg.len() as u8;
            i += 1;
        } else {
            self.memory[i] = 0xFF;
            self.memory[i + 1..i + 3].copy_from_slice(&(msg.len() as u16).to_be_bytes());
            i += 3;
        }
        self.memory[i..i + msg.len()].copy_from_slice(msg);
        i += msg.len();
        self.memory[i] = 0xFE;
        i += 1;
        for b in &mut self.memory[i..(USER_LAST_PAGE + 1) * 4] {
            *b = 0;
        }
        true
    }

    /// The NDEF message currently in user memory (first NDEF TLV), if any.
    pub fn ndef(&self) -> Option<&[u8]> {
        let user = &self.memory[USER_FIRST_PAGE * 4..(USER_LAST_PAGE + 1) * 4];
        let mut i = 0;
        while i < user.len() {
            match user[i] {
                0x00 => i += 1, // NULL TLV
                0xFE => return None,
                t => {
                    let (len, hdr) = if user.get(i + 1)? == &0xFF {
                        (u16::from_be_bytes([*user.get(i + 2)?, *user.get(i + 3)?]) as usize, 4)
                    } else {
                        (*user.get(i + 1)? as usize, 2)
                    };
                    let v = user.get(i + hdr..i + hdr + len)?;
                    if t == 0x03 {
                        return Some(v);
                    }
                    i += hdr + len;
                }
            }
        }
        None
    }

    // ------------------------------------------------------------------
    // chip control
    // ------------------------------------------------------------------

    /// Re-initialise the ST25R3916 (chip defaults, no reader-mode receiver
    /// tuning), configure it as an NFC-A passive target and enter `Off`.
    pub fn start<I: I2c>(
        &mut self,
        nfc: &mut St25r3916<I>,
        delay: &mut impl DelayNs,
    ) -> Result<(), crate::st25r3916::Error<I::Error>> {
        nfc.init(delay)?;
        self.configure(nfc, delay)?;
        Ok(())
    }

    fn configure<I: I2c>(&mut self, nfc: &mut St25r3916<I>, delay: &mut impl DelayNs) -> Result<(), I::Error> {
        nfc.command(cmd::STOP_ALL)?;
        nfc.modify_reg(reg::OP_CONTROL, OP_WU, 0)?;
        // External field detector in automatic mode: this is what wakes us
        // (I_eon / efd_o) while the chip sits in low-power Off.
        nfc.modify_reg(reg::OP_CONTROL, 0x03, 0x03)?;

        // PT memory A: UID[10], ATQA[2], SAK for cascade levels 1..3.
        let uid = self.uid();
        let mut pt = [0u8; 15];
        pt[..7].copy_from_slice(&uid);
        pt[10] = ATQA[0];
        pt[11] = ATQA[1];
        pt[12] = SAK_CASCADE;
        pt[13] = 0x00;
        pt[14] = 0x00;
        nfc.modify_reg(reg::AUX, 0x30, 0x10)?; // 7-byte UID
        nfc.load_pt_mem_a(&pt)?;

        nfc.write_reg(reg::MODE, MODE_TARGET_BITRATE_DETECT)?;
        // FDT correction 5, no AP2P / 212-424 auto response, NFC-A auto response on.
        nfc.write_reg(reg::NFCIP1_PASSIVE_TARGET, 0x50 | D_AC_AP2P | D_212_424_1R)?;
        nfc.write_reg(reg::IRQ_MASK_TARGET, 0x00)?;
        // Timer control: no GPT trigger, MRT step 512/fc; MRT ~100 us.
        nfc.write_reg(reg::TIMER_EMV_CONTROL, 0x08)?;
        nfc.write_reg(reg::MASK_RX_TIMER, 4)?;
        // ISO14443A: parity on, NFCIP-1 off.
        nfc.modify_reg(reg::ISO14443A_NFC, 0xE0, 0)?;
        nfc.write_reg(reg::ANT_TUNE_A, 0x00)?;
        nfc.write_reg(reg::ANT_TUNE_B, 0xFF)?;
        for r in [reg::B_OVERSHOOT_CONF1, reg::B_OVERSHOOT_CONF2, reg::B_UNDERSHOOT_CONF1, reg::B_UNDERSHOOT_CONF2] {
            nfc.write_reg_b(r, 0x00)?;
        }
        nfc.write_irq_mask(0)?;
        nfc.command(cmd::UNMASK_RECEIVE_DATA)?;

        self.dirty = false;
        self.goto_off(nfc, delay)?;
        Ok(())
    }

    /// Leave target mode (chip back in Ready mode, field off, initiator mode).
    pub fn stop<I: I2c>(&mut self, nfc: &mut St25r3916<I>) -> Result<(), I::Error> {
        nfc.modify_reg(reg::OP_CONTROL, 0, OP_EN)?;
        nfc.modify_reg(reg::OP_CONTROL, OP_TX_EN | OP_RX_EN, 0)?;
        nfc.modify_reg(reg::NFCIP1_PASSIVE_TARGET, 0, D_AC_AP2P | D_212_424_1R | D_106_AC_A)?;
        nfc.write_reg(reg::MODE, 0x00)?;
        self.state = State::Off;
        Ok(())
    }

    fn field_present<I: I2c>(nfc: &mut St25r3916<I>) -> Result<bool, I::Error> {
        Ok(nfc.read_reg(reg::AUX_DISPLAY)? & 0x40 != 0)
    }

    fn goto_off<I: I2c>(&mut self, nfc: &mut St25r3916<I>, delay: &mut impl DelayNs) -> Result<(), I::Error> {
        self.bitrate_106 = false;
        self.wakeup = false;
        nfc.command(cmd::STOP_ALL)?;
        nfc.modify_reg(reg::OP_CONTROL, 0, OP_RX_EN)?;
        nfc.modify_reg(reg::NFCIP1_PASSIVE_TARGET, D_106_AC_A, 0)?; // auto response on
        nfc.command(cmd::GO_TO_SENSE)?;
        nfc.modify_reg(reg::ISO14443A_NFC, 0x20, 0)?;
        nfc.clear_irq()?;
        nfc.modify_reg(reg::AUX, 0, AUX_NO_CRC_RX)?;
        nfc.write_reg(reg::MODE, MODE_TARGET_BITRATE_DETECT)?;
        if Self::field_present(nfc)? {
            return self.goto_idle(nfc, delay);
        }
        // Stay in Ready mode (oscillator on) while waiting for a field; the
        // fully powered-down variant (en=0) did not report I_eon on this board.
        nfc.modify_reg(reg::OP_CONTROL, OP_TX_EN, 0)?;
        nfc.modify_reg(reg::OP_CONTROL, 0, OP_EN | OP_RX_EN)?;
        self.state = State::Off;
        Ok(())
    }

    fn goto_idle<I: I2c>(&mut self, nfc: &mut St25r3916<I>, delay: &mut impl DelayNs) -> Result<(), I::Error> {
        let v = nfc.read_reg(reg::OP_CONTROL)?;
        if v & OP_EN == 0 {
            nfc.modify_reg(reg::OP_CONTROL, 0, OP_EN | OP_RX_EN)?;
            let mut waited = 0;
            while nfc.read_reg(reg::AUX_DISPLAY)? & 0x10 == 0 {
                if waited > 200 {
                    return self.goto_off(nfc, delay);
                }
                delay.delay_ms(5);
                waited += 5;
            }
        }
        nfc.modify_reg(reg::AUX, 0, AUX_NO_CRC_RX)?;
        if self.state == State::Active && !self.wakeup {
            nfc.modify_reg(reg::NFCIP1_PASSIVE_TARGET, D_106_AC_A, 0)?;
            nfc.command(cmd::GO_TO_SENSE)?;
        }
        nfc.command(cmd::CLEAR_FIFO)?;
        nfc.command(cmd::UNMASK_RECEIVE_DATA)?;
        self.wakeup = false;
        self.state = State::Idle;
        Ok(())
    }

    fn goto_ready<I: I2c>(&mut self, nfc: &mut St25r3916<I>, delay: &mut impl DelayNs) -> Result<(), I::Error> {
        if nfc.take_irq(irq::EOF)? != 0 {
            return self.goto_off(nfc, delay);
        }
        nfc.modify_reg(reg::AUX, AUX_NO_CRC_RX, 0)?;
        nfc.write_reg(reg::BIT_RATE, 0x00)?;
        nfc.modify_reg(reg::OP_CONTROL, OP_WU, 0)?;
        nfc.write_reg(reg::MODE, MODE_TARGET_LISTEN_A)?;
        self.state = State::Ready;
        Ok(())
    }

    fn goto_active<I: I2c>(&mut self, nfc: &mut St25r3916<I>) -> Result<(), I::Error> {
        nfc.modify_reg(reg::NFCIP1_PASSIVE_TARGET, 0, D_106_AC_A)?; // auto response off
        let _ = nfc.take_irq(irq::ERROR_MASK)?;
        self.state = State::Active;
        Ok(())
    }

    fn goto_halt<I: I2c>(&mut self, nfc: &mut St25r3916<I>, delay: &mut impl DelayNs) -> Result<(), I::Error> {
        nfc.modify_reg(reg::NFCIP1_PASSIVE_TARGET, D_106_AC_A, 0)?;
        nfc.command(cmd::GO_TO_SLEEP)?;
        nfc.write_reg(reg::MODE, MODE_TARGET_BITRATE_DETECT)?;
        nfc.modify_reg(reg::ISO14443A_NFC, 0x20, 0)?;
        nfc.command(cmd::UNMASK_RECEIVE_DATA)?;
        if !Self::field_present(nfc)? {
            return self.goto_off(nfc, delay);
        }
        self.state = State::Halt;
        Ok(())
    }

    fn read_bitrate<I: I2c>(&mut self, nfc: &mut St25r3916<I>) -> Result<(), I::Error> {
        let br = (nfc.read_reg(reg::BITRATE_DETECT)? >> 4) & 0x03;
        self.bitrate_106 = br == 0;
        Ok(())
    }

    /// Drive the state machine; call every few milliseconds while emulating.
    pub fn update<I: I2c>(&mut self, nfc: &mut St25r3916<I>, delay: &mut impl DelayNs) -> Result<Event, I::Error> {
        let before = self.state;
        match self.state {
            State::Off => {
                if nfc.take_irq(irq::EON)? != 0 || Self::field_present(nfc)? {
                    self.commands = 0;
                    self.goto_idle(nfc, delay)?;
                }
            }
            State::Idle | State::Halt => {
                let f = nfc.take_irq(irq::NFCT | irq::RXE | irq::EOF | irq::RXE_PTA)?;
                if f & irq::NFCT != 0 {
                    self.read_bitrate(nfc)?;
                }
                if f & irq::EOF != 0 {
                    self.goto_off(nfc, delay)?;
                } else if f & irq::RXE != 0 {
                    // Frames not handled by the auto-responder: discard.
                    let errs = nfc.take_irq(irq::ERROR_MASK)?;
                    let (bytes, bits) = nfc.fifo_status()?;
                    let mut peek = [0u8; 4];
                    let n = (bytes as usize).min(4);
                    nfc.read_fifo(&mut peek[..n])?;
                    log::debug!("T2T {:?} stray rx: errs=0x{errs:08X} {bytes}B+{bits}b head={:02X?} br106={}", self.state, &peek[..n], self.bitrate_106);
                    nfc.command(cmd::CLEAR_FIFO)?;
                    nfc.command(cmd::UNMASK_RECEIVE_DATA)?;
                    if self.state == State::Idle {
                        nfc.modify_reg(reg::OP_CONTROL, OP_TX_EN, 0)?;
                    }
                } else if f & irq::RXE_PTA != 0 && self.bitrate_106 {
                    let pta = nfc.read_reg(reg::PT_DISPLAY)? & 0x0F;
                    let threshold = if self.state == State::Halt { 0x09 } else { 0x01 };
                    if pta > threshold {
                        self.wakeup = self.state == State::Halt;
                        self.goto_ready(nfc, delay)?;
                    }
                }
            }
            State::Ready => {
                let wake = if self.wakeup { irq::WU_AX } else { irq::WU_A };
                let f = nfc.take_irq(irq::EOF | wake)?;
                if f & irq::EOF != 0 {
                    self.goto_off(nfc, delay)?;
                } else if f & wake != 0 {
                    log::debug!("T2T wake irq=0x{f:08X} pta=0x{:02X}", nfc.read_reg(reg::PT_DISPLAY)?);
                    self.goto_active(nfc)?;
                }
            }
            State::Active => {
                let f = nfc.take_irq(irq::EOF | irq::RXE)?;
                if f & irq::EOF != 0 {
                    self.goto_off(nfc, delay)?;
                } else if f & irq::RXE != 0 {
                    let errs = nfc.take_irq(irq::ERROR_MASK)?;
                    let (bytes, _) = nfc.fifo_status()?;
                    if errs != 0 || bytes <= 2 || bytes as usize > 64 {
                        let mut peek = [0u8; 4];
                        let n = (bytes as usize).min(4);
                        nfc.read_fifo(&mut peek[..n])?;
                        log::debug!("T2T drop: errs=0x{errs:08X} bytes={bytes} head={:02X?}", &peek[..n]);
                        nfc.command(cmd::CLEAR_FIFO)?;
                        nfc.command(cmd::UNMASK_RECEIVE_DATA)?;
                        if self.wakeup {
                            self.goto_halt(nfc, delay)?;
                        } else {
                            self.goto_idle(nfc, delay)?;
                        }
                    } else {
                        let mut rx = [0u8; 64];
                        let n = bytes as usize - 2; // strip CRC
                        nfc.read_fifo(&mut rx[..n])?;
                        self.commands += 1;
                        self.last_cmd = rx[0];
                        log::debug!("T2T cmd: {:02X?}", &rx[..n]);
                        match self.handle_command(nfc, &rx[..n])? {
                            CmdResult::Stay => {}
                            CmdResult::Halt => self.goto_halt(nfc, delay)?,
                            CmdResult::Drop => {
                                if self.wakeup {
                                    self.goto_halt(nfc, delay)?
                                } else {
                                    self.goto_idle(nfc, delay)?
                                }
                            }
                        }
                    }
                }
            }
        }

        let after = self.state;
        Ok(match (before, after) {
            (State::Off, s) if s != State::Off => Event::FieldOn,
            (s, State::Off) if s != State::Off => {
                let written = self.dirty;
                self.dirty = false;
                Event::FieldOff { written }
            }
            (s, State::Active) if s != State::Active => Event::Selected,
            _ => Event::None,
        })
    }

    fn handle_command<I: I2c>(&mut self, nfc: &mut St25r3916<I>, rx: &[u8]) -> Result<CmdResult, I::Error> {
        match rx[0] {
            // READ: 4 pages (16 bytes), wrapping at the end of memory.
            0x30 if rx.len() == 2 => {
                let start = rx[1] as usize;
                if start >= PAGES {
                    nfc.target_transmit(&[NAK], 4)?;
                    return Ok(CmdResult::Drop);
                }
                let mut out = [0u8; 16];
                for (i, o) in out.iter_mut().enumerate() {
                    *o = self.memory[(start * 4 + i) % MEM_BYTES];
                }
                nfc.target_transmit(&out, 0)?;
                Ok(CmdResult::Stay)
            }
            // FAST_READ from..=to pages.
            0x3A if rx.len() == 3 => {
                let (from, to) = (rx[1] as usize, rx[2] as usize);
                if from > to || to >= PAGES || (to - from + 1) * 4 > 512 {
                    nfc.target_transmit(&[NAK], 4)?;
                    return Ok(CmdResult::Drop);
                }
                nfc.target_transmit(&self.memory[from * 4..(to + 1) * 4], 0)?;
                Ok(CmdResult::Stay)
            }
            // WRITE one page.
            0xA2 if rx.len() == 6 => {
                let page = rx[1] as usize;
                if !(2..PAGES).contains(&page) {
                    nfc.target_transmit(&[NAK], 4)?;
                    return Ok(CmdResult::Drop);
                }
                if page >= 3 {
                    self.memory[page * 4..page * 4 + 4].copy_from_slice(&rx[2..6]);
                    if (USER_FIRST_PAGE..=USER_LAST_PAGE).contains(&page) {
                        self.dirty = true;
                    }
                }
                nfc.target_transmit(&[ACK], 4)?;
                Ok(CmdResult::Stay)
            }
            0x60 if rx.len() == 1 => {
                nfc.target_transmit(&GET_VERSION_NTAG216, 0)?;
                Ok(CmdResult::Stay)
            }
            // READ_SIG: 32 zero bytes.
            0x3C if rx.len() == 2 => {
                nfc.target_transmit(&[0u8; 32], 0)?;
                Ok(CmdResult::Stay)
            }
            // HLTA.
            0x50 => Ok(CmdResult::Halt),
            _ => {
                nfc.target_transmit(&[NAK], 4)?;
                Ok(CmdResult::Drop)
            }
        }
    }
}

enum CmdResult {
    Stay,
    Halt,
    Drop,
}
