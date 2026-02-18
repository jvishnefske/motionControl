//! TMC2209 stepper driver UART protocol.
//!
//! Implements the single-wire UART datagram protocol for register
//! reads and writes. Supports multi-driver addressing (0-3) on a
//! shared bus.

/// TMC2209 register addresses.
#[allow(dead_code)]
pub mod reg {
    pub const GCONF: u8 = 0x00;
    pub const GSTAT: u8 = 0x01;
    pub const IFCNT: u8 = 0x02;
    pub const SLAVECONF: u8 = 0x03;
    pub const IOIN: u8 = 0x06;
    pub const IHOLD_IRUN: u8 = 0x10;
    pub const TPOWERDOWN: u8 = 0x11;
    pub const TSTEP: u8 = 0x12;
    pub const TPWMTHRS: u8 = 0x13;
    pub const TCOOLTHRS: u8 = 0x14;
    pub const VACTUAL: u8 = 0x22;
    pub const SGTHRS: u8 = 0x40;
    pub const SG_RESULT: u8 = 0x41;
    pub const COOLCONF: u8 = 0x42;
    pub const MSCNT: u8 = 0x6A;
    pub const CHOPCONF: u8 = 0x6C;
    pub const DRV_STATUS: u8 = 0x6F;
    pub const PWMCONF: u8 = 0x70;
}

/// A write datagram to send to the TMC2209.
#[derive(Clone, Debug)]
pub struct WriteDatagram {
    pub bytes: [u8; 8],
}

/// A read request datagram.
#[derive(Clone, Debug)]
pub struct ReadRequest {
    pub bytes: [u8; 4],
}

/// CRC8-ATM polynomial 0x07, init 0x00.
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &byte in data {
        let mut b = byte;
        for _ in 0..8 {
            if ((crc >> 7) ^ (b & 0x01)) != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
            b >>= 1;
        }
    }
    crc
}

impl WriteDatagram {
    /// Build a write datagram for the given slave address, register, and 32-bit value.
    pub fn new(slave_addr: u8, register: u8, value: u32) -> Self {
        let mut bytes = [0u8; 8];
        bytes[0] = 0x05; // sync
        bytes[1] = slave_addr;
        bytes[2] = register | 0x80; // write flag
        bytes[3] = (value >> 24) as u8;
        bytes[4] = (value >> 16) as u8;
        bytes[5] = (value >> 8) as u8;
        bytes[6] = value as u8;
        bytes[7] = crc8(&bytes[..7]);
        Self { bytes }
    }
}

impl ReadRequest {
    /// Build a read request for the given slave address and register.
    pub fn new(slave_addr: u8, register: u8) -> Self {
        let mut bytes = [0u8; 4];
        bytes[0] = 0x05; // sync
        bytes[1] = slave_addr;
        bytes[2] = register & 0x7F; // read (bit 7 clear)
        bytes[3] = crc8(&bytes[..3]);
        Self { bytes }
    }
}

/// Parse a read reply (8 bytes) into a 32-bit register value.
/// Returns `None` if sync byte or CRC is invalid.
pub fn parse_reply(reply: &[u8; 8]) -> Option<u32> {
    if reply[0] != 0x05 {
        return None;
    }
    let expected_crc = crc8(&reply[..7]);
    if reply[7] != expected_crc {
        return None;
    }
    let value = (reply[3] as u32) << 24
        | (reply[4] as u32) << 16
        | (reply[5] as u32) << 8
        | (reply[6] as u32);
    Some(value)
}

/// Compute the IHOLD_IRUN register value from milliamps.
/// Rsense = 0.056 ohm (physical) + 0.020 internal = 0.076 ohm effective.
pub fn current_to_irun(current_ma: u16) -> u8 {
    const RSENSE: f32 = 0.076;
    const VFS: f32 = 0.180; // vsense=1

    let i_rms = current_ma as f32 / 1000.0;
    let cs = (32.0 * 1.41421 * i_rms * RSENSE / VFS) as i32 - 1;
    cs.clamp(0, 31) as u8
}

/// Build the IHOLD_IRUN register value.
pub fn ihold_irun(irun: u8, ihold: u8, iholddelay: u8) -> u32 {
    ((iholddelay as u32 & 0x0F) << 16)
        | ((irun as u32 & 0x1F) << 8)
        | (ihold as u32 & 0x1F)
}

/// Build CHOPCONF register value for given microstepping.
pub fn chopconf(microsteps: u16, interpolation: bool) -> u32 {
    let mres = match microsteps {
        256 => 0,
        128 => 1,
        64 => 2,
        32 => 3,
        16 => 4,
        8 => 5,
        4 => 6,
        2 => 7,
        _ => 8, // fullstep
    };

    let mut val: u32 = 0;
    val |= 5; // toff=5
    val |= 4 << 4; // hstrt=4
    val |= 2 << 15; // tbl=2
    val |= 1 << 17; // vsense=1 (low-scale 180mV)
    val |= (mres & 0x0F) as u32 >> 0 << 24; // mres
    if interpolation {
        val |= 1 << 28; // intpol
    }
    val
}

/// Build GCONF register for UART mode with optional StealthChop.
pub fn gconf(stealthchop: bool) -> u32 {
    let mut val: u32 = 0;
    val |= 1 << 6; // pdn_disable (enable UART)
    val |= 1 << 7; // mstep_reg_select (microstepping via register)
    val |= 1 << 8; // multistep_filt
    if !stealthchop {
        val |= 1 << 2; // en_spreadcycle
    }
    val
}

/// Build PWMCONF for StealthChop auto-tuning.
pub fn pwmconf_default() -> u32 {
    let mut val: u32 = 0;
    val |= 36; // pwm_ofs = 36
    val |= 14 << 8; // pwm_grad = 14
    val |= 1 << 16; // pwm_freq = 1
    val |= 1 << 18; // pwm_autoscale = 1
    val |= 1 << 19; // pwm_autograd = 1
    val |= 8 << 24; // pwm_reg = 8
    val |= 12 << 28; // pwm_lim = 12
    val
}

/// Complete initialization sequence for one TMC2209 driver.
/// Returns the sequence of (register, value) pairs to write.
pub fn init_sequence(
    current_ma: u16,
    microsteps: u16,
    interpolation: bool,
    stealthchop: bool,
    stallguard_threshold: u8,
) -> [(u8, u32); 8] {
    let irun = current_to_irun(current_ma);
    let ihold = irun / 2;

    [
        (reg::GCONF, gconf(stealthchop)),
        (reg::GSTAT, 0x07), // clear status flags
        (reg::IHOLD_IRUN, ihold_irun(irun, ihold, 8)),
        (reg::TPOWERDOWN, 20),
        (reg::TPWMTHRS, 0), // stealthchop at all velocities
        (reg::CHOPCONF, chopconf(microsteps, interpolation)),
        (reg::PWMCONF, pwmconf_default()),
        (reg::SGTHRS, stallguard_threshold as u32),
    ]
}
