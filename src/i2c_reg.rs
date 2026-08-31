//! Register access helpers shared by the I2C peripheral drivers.

use embedded_hal::i2c::{I2c, Operation};

/// Read `buf.len()` bytes starting at register `reg`.
pub fn read<I: I2c>(i2c: &mut I, addr: u8, reg: u8, buf: &mut [u8]) -> Result<(), I::Error> {
    i2c.write_read(addr, &[reg], buf)
}

/// Read a single 8-bit register.
pub fn read_u8<I: I2c>(i2c: &mut I, addr: u8, reg: u8) -> Result<u8, I::Error> {
    let mut b = [0u8; 1];
    read(i2c, addr, reg, &mut b)?;
    Ok(b[0])
}

/// Read a little-endian 16-bit value from `reg`, `reg+1`.
pub fn read_u16_le<I: I2c>(i2c: &mut I, addr: u8, reg: u8) -> Result<u16, I::Error> {
    let mut b = [0u8; 2];
    read(i2c, addr, reg, &mut b)?;
    Ok(u16::from_le_bytes(b))
}

/// Write `data` starting at register `reg` (auto-increment on the device side).
///
/// Uses a two-part transaction so no intermediate buffer is required; the
/// embedded-hal contract merges consecutive writes into one bus transfer.
pub fn write<I: I2c>(i2c: &mut I, addr: u8, reg: u8, data: &[u8]) -> Result<(), I::Error> {
    i2c.transaction(addr, &mut [Operation::Write(&[reg]), Operation::Write(data)])
}

/// Write a single 8-bit register.
pub fn write_u8<I: I2c>(i2c: &mut I, addr: u8, reg: u8, value: u8) -> Result<(), I::Error> {
    i2c.write(addr, &[reg, value])
}

/// Read-modify-write: `reg = (reg & !mask) | (value & mask)`.
pub fn update_u8<I: I2c>(i2c: &mut I, addr: u8, reg: u8, mask: u8, value: u8) -> Result<(), I::Error> {
    let cur = read_u8(i2c, addr, reg)?;
    let new = (cur & !mask) | (value & mask);
    if new != cur {
        write_u8(i2c, addr, reg, new)?;
    }
    Ok(())
}

/// Set the bits in `mask`.
pub fn set_bits<I: I2c>(i2c: &mut I, addr: u8, reg: u8, mask: u8) -> Result<(), I::Error> {
    update_u8(i2c, addr, reg, mask, mask)
}

/// Clear the bits in `mask`.
pub fn clear_bits<I: I2c>(i2c: &mut I, addr: u8, reg: u8, mask: u8) -> Result<(), I::Error> {
    update_u8(i2c, addr, reg, mask, 0)
}
