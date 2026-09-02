//! ST25R3916 NFC reader driver (I2C, polling) — ISO14443A (NFC-A) UID reading.
//!
//! Scope of this first version:
//! * chip bring-up (oscillator, regulators, analog settings) for a 3.3 V I2C
//!   setup, following the sequence used by M5Unit-NFC / ST RFAL
//! * field on/off
//! * REQA / WUPA, cascade anticollision (levels 1..3) and SELECT → UID + SAK
//! * generic `transceive` for higher layers (Type 2 READ etc.)
//!
//! Interrupts are polled through the interrupt registers; the IRQ pin is not
//! required.
//!
//! ## I2C framing
//! First byte after the address selects the operation:
//! `00aa_aaaa` write register, `01aa_aaaa` read register, `0x80` load FIFO,
//! `0x9F` read FIFO, `11cc_cccc` direct command. Space-B registers are
//! addressed by prefixing `0xFB`; test registers by `0xFC`.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{I2c, Operation};

pub const ADDR: u8 = 0x50;
pub const IC_TYPE_ST25R3916: u8 = 0x05;
pub const FIFO_DEPTH: usize = 512;

/// Register addresses (space A unless noted).
pub mod reg {
    pub const IO_CONF1: u8 = 0x00;
    pub const IO_CONF2: u8 = 0x01;
    pub const OP_CONTROL: u8 = 0x02;
    pub const MODE: u8 = 0x03;
    pub const BIT_RATE: u8 = 0x04;
    pub const ISO14443A_NFC: u8 = 0x05;
    pub const NFCIP1_PASSIVE_TARGET: u8 = 0x08;
    pub const AUX: u8 = 0x0A;
    pub const RX_CONF1: u8 = 0x0B;
    pub const RX_CONF2: u8 = 0x0C;
    pub const RX_CONF3: u8 = 0x0D;
    pub const RX_CONF4: u8 = 0x0E;
    pub const MASK_RX_TIMER: u8 = 0x0F;
    pub const NO_RESPONSE_TIMER1: u8 = 0x10;
    pub const TIMER_EMV_CONTROL: u8 = 0x12;
    pub const IRQ_MASK_MAIN: u8 = 0x16;
    pub const IRQ_MAIN: u8 = 0x1A;
    pub const IRQ_TIMER_NFC: u8 = 0x1B;
    pub const IRQ_ERROR_WUP: u8 = 0x1C;
    pub const IRQ_TARGET: u8 = 0x1D;
    pub const FIFO_STATUS1: u8 = 0x1E;
    pub const FIFO_STATUS2: u8 = 0x1F;
    pub const COLLISION_STATUS: u8 = 0x20;
    pub const NUM_TX_BYTES1: u8 = 0x22;
    pub const ANT_TUNE_A: u8 = 0x26;
    pub const ANT_TUNE_B: u8 = 0x27;
    pub const TX_DRIVER: u8 = 0x28;
    pub const PT_MOD: u8 = 0x29;
    pub const FIELD_THRESHOLD_ACT: u8 = 0x2A;
    pub const FIELD_THRESHOLD_DEACT: u8 = 0x2B;
    pub const AUX_DISPLAY: u8 = 0x31;
    pub const IC_IDENTITY: u8 = 0x3F;
    pub const PT_DISPLAY: u8 = 0x21;
    pub const BITRATE_DETECT: u8 = 0x24;
    pub const IRQ_MASK_TARGET: u8 = 0x19;
    // space B
    pub const B_EMD_SUP_CONF: u8 = 0x05;
    pub const B_CORR_CONF1: u8 = 0x0C;
    pub const B_CORR_CONF2: u8 = 0x0D;
    pub const B_RES_AM_MOD: u8 = 0x2A;
    pub const B_OVERSHOOT_CONF1: u8 = 0x30;
    pub const B_OVERSHOOT_CONF2: u8 = 0x31;
    pub const B_UNDERSHOOT_CONF1: u8 = 0x32;
    pub const B_UNDERSHOOT_CONF2: u8 = 0x33;
}

pub mod cmd {
    pub const SET_DEFAULT: u8 = 0xC1;
    pub const GO_TO_SENSE: u8 = 0xCD;
    pub const GO_TO_SLEEP: u8 = 0xCE;
    pub const UNMASK_RECEIVE_DATA: u8 = 0xD1;
    pub const STOP_ALL: u8 = 0xC2;
    pub const TRANSMIT_WITH_CRC: u8 = 0xC4;
    pub const TRANSMIT_WITHOUT_CRC: u8 = 0xC5;
    pub const TRANSMIT_REQA: u8 = 0xC6;
    pub const TRANSMIT_WUPA: u8 = 0xC7;
    pub const INITIAL_RF_COLLISION: u8 = 0xC8;
    pub const RESET_RX_GAIN: u8 = 0xD5;
    pub const ADJUST_REGULATORS: u8 = 0xD6;
    pub const CLEAR_FIFO: u8 = 0xDB;
    pub const SPACE_B_ACCESS: u8 = 0xFB;
    pub const TEST_ACCESS: u8 = 0xFC;
}

/// Bits of the 32-bit interrupt word: `main<<24 | timer_nfc<<16 | error<<8 | target`.
pub mod irq {
    pub const OSC: u32 = 0x80 << 24;
    pub const WL: u32 = 0x40 << 24;
    pub const RXS: u32 = 0x20 << 24;
    pub const RXE: u32 = 0x10 << 24;
    pub const TXE: u32 = 0x08 << 24;
    pub const COL: u32 = 0x04 << 24;
    pub const NRE: u32 = 0x40 << 16;
    pub const CRC: u32 = 0x80 << 8;
    pub const PAR: u32 = 0x40 << 8;
    pub const ERR2: u32 = 0x20 << 8;
    pub const ERR1: u32 = 0x10 << 8;
    /// Timer/NFC register: external field detected / dropped, bit rate found.
    pub const EON: u32 = 0x10 << 16;
    pub const EOF: u32 = 0x08 << 16;
    pub const NFCT: u32 = 0x01 << 16;
    /// Passive target register.
    pub const RXE_PTA: u32 = 0x10;
    pub const WU_AX: u32 = 0x02;
    pub const WU_A: u32 = 0x01;
    /// Communication errors only (CRC, parity, framing); the low nibble of
    /// the error register holds wake-up flags, which are not errors.
    pub const ERROR_MASK: u32 = 0x0000_F000;
}

mod op {
    pub const EN: u8 = 0x80;
    pub const RX_EN: u8 = 0x40;
    pub const TX_EN: u8 = 0x08;
    pub const WU: u8 = 0x04;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    I2c(E),
    /// IC identity register did not report an ST25R3916.
    NotFound { identity: u8 },
    /// Oscillator did not report stable.
    Oscillator,
    /// No response within the timeout.
    Timeout,
    /// Receive error flagged by the chip (CRC / parity / framing).
    RxError(u32),
    /// Unexpected frame length/content.
    Protocol,
    /// Anticollision did not converge.
    Collision,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::I2c(e)
    }
}

/// Result of a successful NFC-A activation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct NfcaTag {
    pub atqa: u16,
    pub uid: [u8; 10],
    pub uid_len: u8,
    pub sak: u8,
}

impl NfcaTag {
    pub fn uid(&self) -> &[u8] {
        &self.uid[..self.uid_len as usize]
    }
    /// SAK bit 6 set → ISO14443-4 compliant (e.g. DESFire, smartphones).
    pub fn is_iso14443_4(&self) -> bool {
        self.sak & 0x20 != 0
    }
    /// Random (per-activation) UID, as used by smartphones: 4-byte UID
    /// starting with 0x08. Such UIDs change on every activation, so identity
    /// comparisons across polls are meaningless.
    pub fn has_random_uid(&self) -> bool {
        self.uid_len == 4 && self.uid[0] == 0x08
    }
}

pub struct St25r3916<I2C> {
    i2c: I2C,
    addr: u8,
    /// Interrupt flags read from the chip but not yet consumed.
    pending_irq: u32,
    /// Flags observed by the last `nfca_request` (diagnostics).
    pub last_request_irq: u32,
}

impl<I2C: I2c> St25r3916<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self { i2c, addr: ADDR, pending_irq: 0, last_request_irq: 0 }
    }

    pub fn release(self) -> I2C {
        self.i2c
    }

    // ------------------------------------------------------------------
    // raw access
    // ------------------------------------------------------------------

    pub fn read_reg(&mut self, r: u8) -> Result<u8, I2C::Error> {
        let mut b = [0u8; 1];
        self.i2c.write_read(self.addr, &[0x40 | (r & 0x3F)], &mut b)?;
        Ok(b[0])
    }

    pub fn read_regs(&mut self, r: u8, buf: &mut [u8]) -> Result<(), I2C::Error> {
        self.i2c.write_read(self.addr, &[0x40 | (r & 0x3F)], buf)
    }

    pub fn write_reg(&mut self, r: u8, v: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, &[r & 0x3F, v])
    }

    pub fn write_regs(&mut self, r: u8, data: &[u8]) -> Result<(), I2C::Error> {
        self.i2c.transaction(self.addr, &mut [Operation::Write(&[r & 0x3F]), Operation::Write(data)])
    }

    pub fn modify_reg(&mut self, r: u8, clear: u8, set: u8) -> Result<(), I2C::Error> {
        let v = self.read_reg(r)?;
        let n = (v & !clear) | set;
        if n != v {
            self.write_reg(r, n)?;
        }
        Ok(())
    }

    pub fn read_reg_b(&mut self, r: u8) -> Result<u8, I2C::Error> {
        let mut b = [0u8; 1];
        self.i2c.write_read(self.addr, &[cmd::SPACE_B_ACCESS, 0x40 | (r & 0x3F)], &mut b)?;
        Ok(b[0])
    }

    pub fn write_reg_b(&mut self, r: u8, v: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, &[cmd::SPACE_B_ACCESS, r & 0x3F, v])
    }

    pub fn command(&mut self, c: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, &[c])
    }

    pub fn load_fifo(&mut self, data: &[u8]) -> Result<(), I2C::Error> {
        self.i2c.transaction(self.addr, &mut [Operation::Write(&[0x80]), Operation::Write(data)])
    }

    /// (bytes, extra bits of last byte) currently in the FIFO.
    pub fn fifo_status(&mut self) -> Result<(u16, u8), I2C::Error> {
        let mut s = [0u8; 2];
        self.read_regs(reg::FIFO_STATUS1, &mut s)?;
        let bytes = s[0] as u16 | (((s[1] & 0xC0) as u16) << 2);
        let bits = (s[1] >> 1) & 0x07;
        Ok((bytes, bits))
    }

    pub fn read_fifo(&mut self, buf: &mut [u8]) -> Result<(), I2C::Error> {
        if buf.is_empty() {
            return Ok(());
        }
        self.i2c.write_read(self.addr, &[0x9F], buf)
    }

    /// Read (and thereby clear) all four interrupt registers.
    pub fn read_irq(&mut self) -> Result<u32, I2C::Error> {
        // The error register must be read first: reading the main register
        // resets it (per M5Unit-NFC).
        let err = self.read_reg(reg::IRQ_ERROR_WUP)?;
        let mut mn = [0u8; 2];
        self.read_regs(reg::IRQ_MAIN, &mut mn)?;
        let tgt = self.read_reg(reg::IRQ_TARGET)?;
        Ok(((mn[0] as u32) << 24) | ((mn[1] as u32) << 16) | ((err as u32) << 8) | tgt as u32)
    }

    pub fn clear_irq(&mut self) -> Result<(), I2C::Error> {
        self.pending_irq = 0;
        let mut d = [0u8; 4];
        self.read_regs(reg::IRQ_MAIN, &mut d)
    }

    pub fn write_irq_mask(&mut self, mask: u32) -> Result<(), I2C::Error> {
        self.write_regs(reg::IRQ_MASK_MAIN, &mask.to_be_bytes())
    }

    /// Fold fresh flags into the pending set and return the whole set
    /// WITHOUT consuming anything (safe for diagnostics/heartbeats).
    pub fn peek_irq(&mut self) -> Result<u32, I2C::Error> {
        self.pending_irq |= self.read_irq()?;
        Ok(self.pending_irq)
    }

    /// Non-blocking: fold fresh interrupt flags into the pending set and
    /// return (and consume) the ones matching `bits`.
    pub fn take_irq(&mut self, bits: u32) -> Result<u32, I2C::Error> {
        self.pending_irq |= self.read_irq()?;
        let hit = self.pending_irq & bits;
        self.pending_irq &= !hit;
        Ok(hit)
    }

    /// Poll the interrupt registers until any of `bits` is set or `timeout_ms`
    /// elapses. Returns the accumulated flags (with `NRE` forced on timeout).
    fn wait_irq(&mut self, delay: &mut impl DelayNs, bits: u32, timeout_ms: u32) -> Result<u32, I2C::Error> {
        let mut elapsed = 0u32;
        loop {
            self.pending_irq |= self.read_irq()?;
            let hit = self.pending_irq & bits;
            if hit != 0 {
                let out = self.pending_irq;
                self.pending_irq &= !hit;
                return Ok(out);
            }
            if elapsed >= timeout_ms * 4 {
                return Ok(self.pending_irq | irq::NRE);
            }
            delay.delay_us(250);
            elapsed += 1;
        }
    }

    // ------------------------------------------------------------------
    // bring-up
    // ------------------------------------------------------------------

    pub fn identity(&mut self) -> Result<u8, I2C::Error> {
        self.read_reg(reg::IC_IDENTITY)
    }

    /// Full power-up sequence for I2C @ 400 kHz, VDD = 3.3 V. Leaves the
    /// chip in Ready mode with the field off.
    pub fn init(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I2C::Error>> {
        let mut identity = 0;
        let mut ok = false;
        for _ in 0..5 {
            if let Ok(v) = self.identity() {
                identity = v;
                if (v >> 3) & 0x1F == IC_TYPE_ST25R3916 && v & 0x07 != 0 {
                    ok = true;
                    break;
                }
            }
            delay.delay_ms(20);
        }
        if !ok {
            return Err(Error::NotFound { identity });
        }

        // Defensive stop of anything left running.
        self.command(cmd::STOP_ALL)?;
        self.modify_reg(reg::OP_CONTROL, op::TX_EN | op::RX_EN, 0)?;
        delay.delay_ms(2);

        self.command(cmd::SET_DEFAULT)?;
        // Overheat-protection workaround (test register 0x04 = 0x10).
        self.i2c.write(self.addr, &[cmd::TEST_ACCESS, 0x04, 0x10])?;

        // IO config: I2C thd for 400 kHz; io_drv_lvl; 3.3 V supply.
        self.write_regs(reg::IO_CONF1, &[0x10, 0x04 | 0x80])?;
        // TX driver: AM modulation index code 13.
        self.write_reg(reg::TX_DRIVER, 13 << 4)?;
        // MCU_CLK disabled.
        self.modify_reg(reg::IO_CONF1, 0x07, 0x07)?;
        // AAT DAC enable dance.
        self.write_reg_b(reg::B_RES_AM_MOD, 0x80)?;
        self.modify_reg(reg::IO_CONF2, 0, 0x20)?;
        self.write_reg_b(reg::B_RES_AM_MOD, 0x00)?;
        // External field detector thresholds.
        self.write_reg(reg::FIELD_THRESHOLD_ACT, 0x13)?;
        self.write_reg(reg::FIELD_THRESHOLD_DEACT, 0x02)?;
        // FDT correction.
        self.modify_reg(reg::NFCIP1_PASSIVE_TARGET, 0xF0, 0x50)?;
        self.write_reg(reg::PT_MOD, 0x5F)?;
        self.write_reg_b(reg::B_EMD_SUP_CONF, 0x40)?;
        self.write_reg(reg::ANT_TUNE_A, 0x82)?;
        self.write_reg(reg::ANT_TUNE_B, 0x82)?;
        // External field detector off: pure reader, field controlled by tx_en.
        self.modify_reg(reg::OP_CONTROL, 0x03, 0)?;
        self.command(cmd::CLEAR_FIFO)?;

        // Mask everything except the error group, then start the oscillator.
        self.write_irq_mask(0xFFFF_00FF)?;
        self.clear_irq()?;
        self.enable_osc(delay)?;
        self.write_irq_mask(0)?;

        self.command(cmd::ADJUST_REGULATORS)?;
        delay.delay_ms(5);
        Ok(())
    }

    fn enable_osc(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I2C::Error>> {
        let v = self.read_reg(reg::OP_CONTROL)?;
        if v & op::EN == 0 {
            self.modify_reg(reg::IRQ_MASK_MAIN, 0x80, 0)?; // unmask I_osc
            self.clear_irq()?;
            self.modify_reg(reg::OP_CONTROL, 0, op::EN)?;
            let flags = self.wait_irq(delay, irq::OSC, 50)?;
            self.modify_reg(reg::IRQ_MASK_MAIN, 0, 0x80)?;
            if flags & irq::OSC == 0 && self.read_reg(reg::AUX_DISPLAY)? & 0x10 == 0 {
                return Err(Error::Oscillator);
            }
        }
        if self.read_reg(reg::AUX_DISPLAY)? & 0x10 == 0 {
            return Err(Error::Oscillator);
        }
        Ok(())
    }

    /// Configure the analog/digital front end for ISO14443A @ 106 kbps and
    /// switch the RF field on.
    pub fn configure_nfca(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I2C::Error>> {
        self.command(cmd::STOP_ALL)?;
        self.modify_reg(reg::OP_CONTROL, op::WU, 0)?;

        self.write_reg(reg::MODE, (0x01 << 3) | 0x01)?; // initiator, ISO14443A, nfc_ar auto
        self.write_reg(reg::BIT_RATE, 0x00)?; // 106/106
        self.write_reg(reg::ISO14443A_NFC, 0x00)?;
        self.modify_reg(reg::AUX, 0x04, 0)?; // correlator on
        self.write_reg_b(reg::B_OVERSHOOT_CONF1, 0x40)?;
        self.write_reg_b(reg::B_OVERSHOOT_CONF2, 0x03)?;
        self.write_reg_b(reg::B_UNDERSHOOT_CONF1, 0x40)?;
        self.write_reg_b(reg::B_UNDERSHOOT_CONF2, 0x03)?;
        self.write_reg_b(reg::B_CORR_CONF1, 0x47)?;
        self.write_reg_b(reg::B_CORR_CONF2, 0x00)?;
        self.write_reg(reg::RX_CONF1, 0x08)?; // z_600k
        self.write_reg(reg::RX_CONF2, 0x20 | 0x08 | 0x04 | 0x01)?; // sqm_dyn, agc_en, agc_m, agc6_3
        self.write_reg(reg::RX_CONF3, 0xD8)?;
        self.write_reg(reg::RX_CONF4, 0x22)?;
        self.command(cmd::RESET_RX_GAIN)?;
        self.write_irq_mask(0)?;
        self.field_on(delay)
    }

    /// Switch the RF field on (reader mode: no collision avoidance, external
    /// field detector off) and wait the ISO14443 guard time.
    pub fn field_on(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I2C::Error>> {
        // Note: AUX_DISPLAY.tx_on is not usable as a field indicator on this
        // board (stays 0 while tags read fine), so trust tx_en.
        let v = self.read_reg(reg::OP_CONTROL)?;
        if v & op::TX_EN != 0 {
            return Ok(());
        }
        // en_fd_c = 00 (manual, FD off) so tx_en drives the field directly.
        self.modify_reg(reg::OP_CONTROL, 0x03, 0)?;
        self.modify_reg(reg::OP_CONTROL, 0, op::TX_EN | op::RX_EN)?;
        delay.delay_ms(6); // guard time before the first REQA
        Ok(())
    }

    /// `true` when AUX_DISPLAY reports tx_on. Note: on the PaperMono this bit
    /// stayed 0 while tags were being read successfully, so do not use it as
    /// a field-present check; it seems to reflect active modulation only.
    pub fn is_field_on(&mut self) -> Result<bool, I2C::Error> {
        Ok(self.read_reg(reg::AUX_DISPLAY)? & 0x20 != 0)
    }

    /// Field on through the chip's initial RF collision avoidance (en_fd = 01),
    /// as RFAL does. Returns the interrupt flags observed.
    pub fn field_on_ca(&mut self, delay: &mut impl DelayNs) -> Result<u32, Error<I2C::Error>> {
        self.modify_reg(reg::OP_CONTROL, op::TX_EN | op::RX_EN, 0)?;
        self.modify_reg(reg::OP_CONTROL, 0x03, 0x01)?; // manual CA
        self.write_reg_b(0x15, 0x00)?; // NFC field-on guard timer
        self.clear_irq()?;
        self.command(cmd::INITIAL_RF_COLLISION)?;
        // I_cac (collision) = 0x04<<16, I_cat (guard time done) = 0x02<<16, I_apon = 0x20
        let flags = self.wait_irq(delay, (0x04 << 16) | (0x02 << 16) | 0x20, 20)?;
        self.modify_reg(reg::OP_CONTROL, 0, op::TX_EN | op::RX_EN)?;
        delay.delay_ms(6);
        Ok(flags)
    }

    /// Read a test-space register (prefix 0xFC).
    pub fn read_test_reg(&mut self, r: u8) -> Result<u8, I2C::Error> {
        let mut b = [0u8; 1];
        self.i2c.write_read(self.addr, &[cmd::TEST_ACCESS, 0x40 | (r & 0x3F)], &mut b)?;
        Ok(b[0])
    }

    /// Run "Measure amplitude" and return the ADC result (0x25).
    pub fn measure_amplitude(&mut self, delay: &mut impl DelayNs) -> Result<u8, I2C::Error> {
        self.command(0xD3)?;
        delay.delay_ms(2);
        self.read_reg(0x25)
    }

    /// Dump space-A registers 0x00..=0x3F into `out`.
    pub fn dump_regs(&mut self, out: &mut [u8; 64]) -> Result<(), I2C::Error> {
        for (i, o) in out.iter_mut().enumerate() {
            *o = self.read_reg(i as u8)?;
        }
        Ok(())
    }

    /// Snapshot of (OP_CONTROL, AUX_DISPLAY, regulator display) for logging.
    /// AUX_DISPLAY: 0x20 tx_on, 0x10 osc_ok, 0x08 rx_on, 0x40 efd_o.
    pub fn status(&mut self) -> Result<(u8, u8, u8), I2C::Error> {
        Ok((self.read_reg(reg::OP_CONTROL)?, self.read_reg(reg::AUX_DISPLAY)?, self.read_reg_b(0x2C)?))
    }

    /// Switch the RF field off (tx_en/rx_en cleared; the chip stays in
    /// Ready mode so [`Self::field_on`] is quick).
    pub fn field_off(&mut self) -> Result<(), I2C::Error> {
        self.modify_reg(reg::OP_CONTROL, op::TX_EN | op::RX_EN, 0)
    }

    /// Ready mode off (oscillator stopped) — lowest power while keeping I2C alive.
    pub fn power_down(&mut self) -> Result<(), I2C::Error> {
        self.field_off()?;
        self.modify_reg(reg::OP_CONTROL, op::EN, 0)
    }

    // ------------------------------------------------------------------
    // timers / framing helpers
    // ------------------------------------------------------------------

    /// Program the no-response timer (frame waiting time) in milliseconds.
    fn set_fwt_ms(&mut self, ms: u32) -> Result<(), I2C::Error> {
        let ctrl = self.read_reg(reg::TIMER_EMV_CONTROL)?;
        let step = if ctrl & 0x01 != 0 { 4096u64 } else { 64u64 };
        // ticks = ceil(ms * 13.56e6 / (step * 1000))
        let ticks = ((ms as u64) * 13_560_000).div_ceil(step * 1000);
        let nrt = ticks.clamp(1, 0xFFFF) as u16;
        self.write_regs(reg::NO_RESPONSE_TIMER1, &nrt.to_be_bytes())
    }

    /// Load the passive-target "A" configuration (UID[10], ATQA[2], SAK[3]).
    pub fn load_pt_mem_a(&mut self, data: &[u8]) -> Result<(), I2C::Error> {
        self.i2c.transaction(self.addr, &mut [Operation::Write(&[0xA0]), Operation::Write(data)])
    }

    /// Transmit `data` from target mode: `bits == 0` → whole bytes with CRC,
    /// otherwise `bits` bits of `data[0]` without CRC (4-bit ACK/NAK).
    pub fn target_transmit(&mut self, data: &[u8], bits: u8) -> Result<(), I2C::Error> {
        self.command(cmd::CLEAR_FIFO)?;
        self.load_fifo(data)?;
        if bits == 0 {
            self.set_num_tx(data.len() as u16, 0)?;
            self.command(cmd::TRANSMIT_WITH_CRC)
        } else {
            self.set_num_tx(0, bits)?;
            self.command(cmd::TRANSMIT_WITHOUT_CRC)
        }
    }

    pub fn set_num_tx(&mut self, bytes: u16, bits: u8) -> Result<(), I2C::Error> {
        let v = ((bytes & 0x1FF) << 3) | (bits as u16 & 0x07);
        self.write_regs(reg::NUM_TX_BYTES1, &v.to_be_bytes())
    }

    fn set_rx_crc_check(&mut self, check: bool) -> Result<(), I2C::Error> {
        // AUX.no_crc_rx = 1 disables CRC checking (and keeps CRC in FIFO).
        self.modify_reg(reg::AUX, 0x80, if check { 0 } else { 0x80 })
    }

    // ------------------------------------------------------------------
    // ISO14443A
    // ------------------------------------------------------------------

    /// Send REQA (`wakeup == false`) or WUPA and return ATQA if a tag answers.
    pub fn nfca_request(&mut self, delay: &mut impl DelayNs, wakeup: bool) -> Result<Option<u16>, Error<I2C::Error>> {
        self.set_fwt_ms(4)?;
        self.write_reg(reg::ISO14443A_NFC, 0x01)?; // antcl
        self.set_rx_crc_check(false)?;
        self.clear_irq()?;
        self.command(cmd::CLEAR_FIFO)?;
        self.command(if wakeup { cmd::TRANSMIT_WUPA } else { cmd::TRANSMIT_REQA })?;

        let mut flags = self.wait_irq(delay, irq::RXE | irq::RXS | irq::COL, 6)?;
        if flags & irq::RXE == 0 && flags & irq::RXS != 0 {
            // Late RXE: give the FIFO a little more time.
            for _ in 0..40 {
                if self.fifo_status()?.0 >= 2 {
                    flags |= irq::RXE;
                    break;
                }
                delay.delay_us(100);
            }
        }
        self.last_request_irq = flags;
        if flags & irq::RXE == 0 {
            return Ok(None);
        }
        let (bytes, _) = self.fifo_status()?;
        if bytes < 2 {
            return Ok(None);
        }
        let mut atqa = [0u8; 2];
        self.read_fifo(&mut atqa)?;
        Ok(Some(u16::from_le_bytes(atqa)))
    }

    /// One cascade level of anticollision. Returns the 5-byte response
    /// (CT/UID bytes + BCC) for `sel` in 0x93/0x95/0x97.
    fn nfca_anticollision_level(&mut self, delay: &mut impl DelayNs, sel: u8) -> Result<[u8; 5], Error<I2C::Error>> {
        self.set_fwt_ms(8)?;
        self.write_reg(reg::ISO14443A_NFC, 0x01)?; // antcl
        self.set_rx_crc_check(true)?;

        let mut frame = [0u8; 7];
        frame[0] = sel;
        frame[1] = 0x20;
        let mut rbuf = [0u8; 5];
        let mut sbytes: usize = 2;
        let mut sbits: u8 = 0;
        let mut rbuf_offset: usize = 0;
        let mut coll_byte: u8 = 1;

        for _ in 0..32 {
            self.clear_irq()?;
            self.command(cmd::CLEAR_FIFO)?;
            self.load_fifo(&frame[..sbytes + (sbits != 0) as usize])?;
            self.set_num_tx(sbytes as u16, sbits)?;
            self.command(cmd::TRANSMIT_WITHOUT_CRC)?;

            let flags = self.wait_irq(delay, irq::RXE | irq::COL, 10)?;
            let collision = flags & irq::COL != 0;
            if !collision && flags & irq::RXE == 0 {
                return Err(if flags & irq::ERROR_MASK != 0 { Error::RxError(flags) } else { Error::Timeout });
            }
            let (bytes, _) = self.fifo_status()?;
            let want = 5 - rbuf_offset;
            let actual = (bytes as usize).min(want);
            if actual == 0 {
                return Err(Error::Protocol);
            }
            self.read_fifo(&mut rbuf[rbuf_offset..rbuf_offset + actual])?;
            let cd = self.read_reg(reg::COLLISION_STATUS)?;

            if collision {
                let cbytes = ((cd >> 4) & 0x0F) as usize;
                let cbits = (cd >> 1) & 0x07;
                coll_byte = rbuf[rbuf_offset + actual - 1] | (1 << cbits);
                sbytes = cbytes + (cbits == 7) as usize;
                sbits = (cbits + 1) & 0x07;
                frame[1] = ((sbytes as u8) << 4) | sbits;
                frame[2 + rbuf_offset..2 + rbuf_offset + actual].copy_from_slice(&rbuf[rbuf_offset..rbuf_offset + actual]);
                frame[sbytes] = coll_byte;
                rbuf_offset = actual - 1;
            }
            if sbits != 0 {
                rbuf[rbuf_offset] = ((rbuf[rbuf_offset] >> sbits) << sbits) | coll_byte;
            }
            if !collision {
                return Ok(rbuf);
            }
        }
        Err(Error::Collision)
    }

    /// Complete anticollision + SELECT for all cascade levels.
    /// Call after a successful [`Self::nfca_request`].
    pub fn nfca_select(&mut self, delay: &mut impl DelayNs, atqa: u16) -> Result<NfcaTag, Error<I2C::Error>> {
        let mut tag = NfcaTag { atqa, ..Default::default() };
        for lv in 1u8..=3 {
            let sel = 0x91 + lv * 2;
            let r = self.nfca_anticollision_level(delay, sel)?;
            let cascade = r[0] == 0x88;
            let uid_part = if cascade { &r[1..4] } else { &r[0..4] };
            let start = ((lv - 1) * 3) as usize;
            tag.uid[start..start + uid_part.len()].copy_from_slice(uid_part);

            // SELECT: SEL 0x70 + 5 bytes, CRC appended by the chip.
            let mut sel_frame = [0u8; 7];
            sel_frame[0] = sel;
            sel_frame[1] = 0x70;
            sel_frame[2..].copy_from_slice(&r);
            let mut sak = [0u8; 3]; // SAK + CRC (CRC stays in FIFO)
            let n = self.transceive(delay, &sel_frame, &mut sak, 8)?;
            if n < 1 {
                return Err(Error::Protocol);
            }
            tag.sak = sak[0];
            if sak[0] & 0x04 == 0 {
                // UID complete.
                tag.uid_len = match lv { 1 => 4, 2 => 7, _ => 10 };
                return Ok(tag);
            }
        }
        Err(Error::Protocol)
    }

    /// Transmit `tx` with CRC and receive into `rx` (CRC checked by the chip,
    /// the two CRC bytes remain in the FIFO and count towards the returned
    /// length). Returns number of bytes read.
    pub fn transceive(
        &mut self,
        delay: &mut impl DelayNs,
        tx: &[u8],
        rx: &mut [u8],
        timeout_ms: u32,
    ) -> Result<usize, Error<I2C::Error>> {
        self.set_fwt_ms(timeout_ms)?;
        self.write_reg(reg::ISO14443A_NFC, 0x00)?;
        self.set_rx_crc_check(true)?;
        self.clear_irq()?;
        self.command(cmd::CLEAR_FIFO)?;
        self.load_fifo(tx)?;
        self.set_num_tx(tx.len() as u16, 0)?;
        self.command(cmd::TRANSMIT_WITH_CRC)?;

        let flags = self.wait_irq(delay, irq::RXE, timeout_ms + 2)?;
        if flags & irq::RXE == 0 {
            return Err(if flags & irq::ERROR_MASK != 0 { Error::RxError(flags) } else { Error::Timeout });
        }
        let (bytes, _) = self.fifo_status()?;
        let n = (bytes as usize).min(rx.len());
        self.read_fifo(&mut rx[..n])?;
        if flags & irq::ERROR_MASK != 0 {
            return Err(Error::RxError(flags));
        }
        Ok(n)
    }

    /// Put the selected tag into HALT.
    pub fn nfca_halt(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I2C::Error>> {
        self.set_fwt_ms(2)?;
        self.write_reg(reg::ISO14443A_NFC, 0x00)?;
        self.set_rx_crc_check(true)?;
        self.clear_irq()?;
        self.command(cmd::CLEAR_FIFO)?;
        self.load_fifo(&[0x50, 0x00])?;
        self.set_num_tx(2, 0)?;
        self.command(cmd::TRANSMIT_WITH_CRC)?;
        let _ = self.wait_irq(delay, irq::TXE, 4)?;
        Ok(())
    }

    /// Convenience: WUPA → anticollision → SELECT. `Ok(None)` when no tag.
    ///
    /// WUPA (not REQA) is used so that tags left in HALT/ACTIVE by a previous
    /// poll keep answering; call [`Self::nfca_halt`] when done with a tag.
    pub fn nfca_poll(&mut self, delay: &mut impl DelayNs) -> Result<Option<NfcaTag>, Error<I2C::Error>> {
        let Some(atqa) = self.nfca_request(delay, true)? else {
            return Ok(None);
        };
        self.nfca_select(delay, atqa).map(Some)
    }

    /// Type 2 tag READ (4 blocks = 16 bytes from `block`).
    pub fn t2t_read(&mut self, delay: &mut impl DelayNs, block: u8) -> Result<[u8; 16], Error<I2C::Error>> {
        let mut buf = [0u8; 18];
        let n = self.transceive(delay, &[0x30, block], &mut buf, 10)?;
        if n < 16 {
            return Err(Error::Protocol);
        }
        let mut out = [0u8; 16];
        out.copy_from_slice(&buf[..16]);
        Ok(out)
    }
}
