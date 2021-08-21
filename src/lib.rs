#![no_std]

pub const CHIP_ADDRESS: u8 = 0x53;

#[allow(dead_code)]
#[derive(Copy, Clone)]
#[allow(non_camel_case_types)]
pub enum Register {
    // Operation mode control, SW reset
    MAIN_CTRL = 0x00,
    // Measure rate and resolution in active mode
    ALS_UVS_MEAS_RATE = 0x04,
    // Analog Gain
    ALS_UVS_GAIN = 0x05,
    // Part and revision IDs
    PART_ID = 0x06,
    // Status
    MAIN_STATUS = 0x07,
    // ALS measurement, LSB
    ALS_DATA_0 = 0x0D,
    // ALS measurement, Middle Byte
    ALS_DATA_1 = 0x0E,
    // ALS measurement, MSB
    ALS_DATA_2 = 0x0F,
    // UVS measurement, LSB
    UVS_DATA_0 = 0x10,
    // UVS measurement, Middle byte
    UVS_DATA_1 = 0x011,
    // UVS measurement, MSB
    UVS_DATA_2 = 0x12,
    // Interrupt config
    INT_CFG = 0x19,
    // Interrupt persist setting
    INT_PST = 0x1A,
    // ALS/UVS interrupt upper thershold, LSB
    ALS_UVS_THRES_UP_0 = 0x21,
    // ALS/UVS interrupt upper thershold, intervening bits
    ALS_UVS_THRES_UP_1 = 0x22,
    // ALS/UVS interrupt upper thershold, MSB
    ALS_UVS_THRES_UP_2 = 0x23,
    // ALS/UVS interrupt lower thershold, LSB
    ALS_UVS_THRES_LOW_0 = 0x24,
    // ALS/UVS interrupt lower thershold, intervening bits
    ALS_UVS_THRES_LOW_1 = 0x25,
    // ALS/UVS interrupt lower thershold, MSB
    ALS_UVS_THRES_LOW_2 = 0x26,
}

impl Register {
    #[inline(always)]
    pub fn bits(self) -> u8 {
        self as u8
    }
}

#[allow(dead_code)]
#[derive(Copy, Clone)]
pub enum MainCtrl {
    // Software Resest
    SoftwareReset = 0b00010000,
    // Ambient Light Sensor mode, sensor enabled
    ALSMode = 0b00000010,
    // Ultra Violet Sensor mode, sensor enabled
    UVSMode = 0b00001010,
    // Ambient Light Sensor Standby
    ALSStandby = 0b00000000,
    // Ultra Violet Sensor Standby
    UVSStandby = 0b00001000,
}

impl MainCtrl {
    #[inline(always)]
    pub fn bits(self) -> u8 {
        self as u8
    }
}

#[allow(dead_code)]
#[derive(Copy, Clone)]
pub enum Resolution {
    TwentyBit = 0b0000,
    NineteenBit = 0b0001,
    EighteenBit = 0b0010,
    SeventeenBit = 0b0011,
    SixteenBit = 0b0100,
    ThirteenBit = 0b0101,
}

impl Resolution {
    #[inline(always)]
    pub fn bits(self) -> u8 {
        self as u8
    }
}

#[allow(dead_code)]
#[derive(Copy, Clone)]
pub enum Rate {
    TwentyFiveMS = 0b0000,
    FiftyMS = 0b0001,
    OneHundredMS = 0b0010,
    TwoHundredMS = 0b0011,
    FiveHundredMS = 0b0100,
    OneThousandMS = 0b0101,
    TwoThousandMS = 0b0110,
}

impl Rate {
    #[inline(always)]
    pub fn bits(self) -> u8 {
        self as u8
    }
}

#[allow(dead_code)]
#[derive(Copy, Clone)]
pub enum Gain {
    One = 0b00000000,
    Three = 0b00000001,
    Six = 0b00000010,
    Nine = 0b00000011,
    Eighteen = 0b00000100,
}

impl Gain {
    #[inline(always)]
    pub fn bits(self) -> u8 {
        self as u8
    }
}

#[allow(dead_code)]
#[derive(Copy, Clone)]
pub enum InterruptConfig {
    ALSIntDisable = 0b00010000,
    ALSIntEnable = 0b00010100,
    UVSIntDisable = 0b00110000,
    UVSIntEnable = 0b00110100,
}

impl InterruptConfig {
    #[inline(always)]
    pub fn bits(self) -> u8 {
        self as u8
    }
}

#[allow(dead_code)]
#[derive(Copy, Clone)]
pub enum InterruptPersist {
    EveryThreshold = 0b00000000,
    TwoConsecutive = 0b00010000,
    SixteenConsecutive = 0b11110000,
}

impl InterruptPersist {
    #[inline(always)]
    pub fn bits(self) -> u8 {
        self as u8
    }
}

#[derive(Debug)]
pub struct Ltr390<I2C> {
    address: u8,
    bus: I2C,
}

impl<I2C, E> Ltr390<I2C>
where
    I2C: embedded_hal::blocking::i2c::Write<Error = E>
        + embedded_hal::blocking::i2c::WriteRead<Error = E>,
{
    #[inline(always)]
    pub fn new(bus: I2C, address: u8) -> Result<Self, E> {
        let ltr390 = Self { address, bus };
        Ok(ltr390)
    }
    #[inline(always)]
    pub fn software_reset(&mut self) -> Result<(), E> {
        self.bus.write(
            self.address,
            &[Register::MAIN_CTRL.bits(), MainCtrl::SoftwareReset.bits()],
        )?;
        Ok(())
    }
    #[inline(always)]
    pub fn set_als_mode(&mut self) -> Result<(), E> {
        self.bus.write(
            self.address,
            &[Register::MAIN_CTRL.bits(), MainCtrl::ALSMode.bits()],
        )?;
        Ok(())
    }
    #[inline(always)]
    pub fn set_uvs_mode(&mut self) -> Result<(), E> {
        self.bus.write(
            self.address,
            &[Register::MAIN_CTRL.bits(), MainCtrl::UVSMode.bits()],
        )?;
        Ok(())
    }
    #[inline(always)]
    pub fn als_standby(&mut self) -> Result<(), E> {
        self.bus.write(
            self.address,
            &[Register::MAIN_CTRL.bits(), MainCtrl::ALSStandby.bits()],
        )?;
        Ok(())
    }
    #[inline(always)]
    pub fn uvs_standby(&mut self) -> Result<(), E> {
        self.bus.write(
            self.address,
            &[Register::MAIN_CTRL.bits(), MainCtrl::UVSStandby.bits()],
        )?;
        Ok(())
    }
    #[inline(always)]
    pub fn main_status(&mut self) -> Result<u8, E> {
        let mut buffer = [0 as u8, 1];
        self.bus
            .write_read(self.address, &[Register::MAIN_STATUS as u8], &mut buffer)?;
        Ok(buffer[0] as u8)
    }
    #[inline(always)]
    pub fn set_als_uvs_meas_rate(&mut self, resolution: Resolution, rate: Rate) -> Result<(), E> {
        let data: u8 = resolution.bits() << 4 | rate.bits();
        self.bus
            .write(self.address, &[Register::ALS_UVS_MEAS_RATE.bits(), data])?;
        Ok(())
    }
    #[inline(always)]
    pub fn read_als_uvs_meas_rate(&mut self) -> Result<u8, E> {
        let mut buffer = [0 as u8, 1];
        self.bus.write_read(
            self.address,
            &[Register::ALS_UVS_MEAS_RATE.bits()],
            &mut buffer,
        )?;
        Ok(buffer[0])
    }
    #[inline(always)]
    pub fn set_als_uvs_gain(&mut self, gain: Gain) -> Result<(), E> {
        let data: u8 = gain.bits();
        self.bus
            .write(self.address, &[Register::ALS_UVS_GAIN.bits(), data])?;
        Ok(())
    }
    #[inline(always)]
    pub fn read_als_uvs_gain(&mut self) -> Result<u8, E> {
        let mut buffer = [0 as u8, 1];
        self.bus
            .write_read(self.address, &[Register::ALS_UVS_GAIN.bits()], &mut buffer)?;
        Ok(buffer[0])
    }
    #[inline(always)]
    pub fn part_id(&mut self) -> Result<[u8; 2], E> {
        let mut buffer = [0 as u8, 1];
        self.bus
            .write_read(self.address, &[Register::PART_ID.bits()], &mut buffer)?;
        let part_number = buffer[0] >> 4;
        let revision_id = buffer[0] & 0x0f;
        Ok([part_number, revision_id])
    }
    #[inline(always)]
    pub fn als_read(&mut self) -> Result<u32, E> {
        let mut buffer = [0 as u8; 3];
        self.bus
            .write_read(self.address, &[Register::ALS_DATA_0.bits()], &mut buffer)?;
        let als_reading: u32 =
            (buffer[2] as u32) << 16 | (buffer[1] as u32) << 8 | buffer[0] as u32;
        Ok(als_reading)
    }
    #[inline(always)]
    pub fn uvs_read(&mut self) -> Result<u32, E> {
        let mut buffer = [0 as u8; 3];
        self.bus
            .write_read(self.address, &[Register::UVS_DATA_0.bits()], &mut buffer)?;
        let uvs_reading: u32 =
            (buffer[2] as u32) << 16 | (buffer[1] as u32) << 8 | buffer[0] as u32;
        Ok(uvs_reading)
    }
    #[inline(always)]
    pub fn interrupt_status(&mut self) -> Result<u8, E>{
        let mut buffer = [0 as u8; 1];
        self.bus.write_read(self.address, &[Register::INT_CFG.bits()], &mut buffer)?;
        Ok(buffer[0])
    }
    #[inline(always)]
    pub fn als_intterupt_enable(&mut self) -> Result<(), E>{
        let data: u8 = InterruptConfig::ALSIntEnable.bits();
        self.bus.write(self.address, &[Register::INT_CFG.bits(), data])?;
        Ok(())
    }
    #[inline(always)]
    pub fn als_intterupt_disable(&mut self) -> Result<(), E>{
        let data: u8 = InterruptConfig::ALSIntDisable.bits();
        self.bus.write(self.address, &[Register::INT_CFG.bits(), data])?;
        Ok(())
    }
    #[inline(always)]
    pub fn uvs_intterupt_enable(&mut self) -> Result<(), E>{
        let data: u8 = InterruptConfig::UVSIntEnable.bits();
        self.bus.write(self.address, &[Register::INT_CFG.bits(), data])?;
        Ok(())
    }
    #[inline(always)]
    pub fn uvs_intterupt_disable(&mut self) -> Result<(), E>{
        let data: u8 = InterruptConfig::UVSIntDisable.bits();
        self.bus.write(self.address, &[Register::INT_CFG.bits(), data])?;
        Ok(())
    }
    #[inline(always)]
    pub fn interrupt_persist(&mut self, rate: u8) -> Result<(), E>{
        match rate{
            0 => self.bus.write(self.address, &[Register::INT_PST.bits(), InterruptPersist::EveryThreshold.bits()])?,
            2 => self.bus.write(self.address, &[Register::INT_PST.bits(), InterruptPersist::TwoConsecutive.bits()])?,
            16 => self.bus.write(self.address, &[Register::INT_PST.bits(), InterruptPersist::SixteenConsecutive.bits()])?,
            _ => self.bus.write(self.address, &[Register::INT_PST.bits(), InterruptPersist::EveryThreshold.bits()])?,
            };
        Ok(())
    }
}
