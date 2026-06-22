#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ReceiverMode {
    Off = 0,
    Rx = 1,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum VendorRequest {
    Reset = 0,
    ReceiverMode = 1,
    ClockgenWrite = 2,
    ClockgenRead = 3,
    RfFrontendWrite = 4,
    RfFrontendRead = 5,
    SpiFlashErase = 6,
    SpiFlashWrite = 7,
    SpiFlashRead = 8,
    BoardIdRead = 9,
    VersionStringRead = 10,
    BoardPartIdSerialNoRead = 11,
    SetSamplerate = 12,
    SetFreq = 13,
    SetLnaGain = 14,
    SetMixerGain = 15,
    SetVgaGain = 16,
    SetLnaAgc = 17,
    SetMixerAgc = 18,
    MsVendorCmd = 19,
    SetRfBiasCmd = 20,
    GpioWrite = 21,
    GpioRead = 22,
    GpioDirWrite = 23,
    GpioDirRead = 24,
    GetSamplerates = 25,
    SetPacking = 26,
    SpiFlashEraseSector = 27,
    SetRfPort = 28,
    GetCapabilities = 29,
    SetBandwidth = 30,
    GetBandwidths = 31,
    GetTemperature = 32,
    SetGain = 33,
    VendorRequestCount = 34,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum RfPort {
    Rx0 = 0,
    Rx1 = 1,
    Rx2 = 2,
    Max = 31,
}

pub const fn rf_ports_mask(n: u32) -> u32 {
    (1u32 << n) - 1
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Capability {
    LnaGain = 0,
    RfGain = 1,
    MixerGain = 2,
    FilterGain = 3,
    VgaGain = 4,
    LnaAgc = 5,
    RfAgc = 6,
    MixerAgc = 7,
    FilterAgc = 8,
    LinearityGain = 9,
    SensitivityGain = 10,
    BiasTee = 11,
    Packing = 12,
    RfPortSelect = 13,
    Gpio = 14,
    SpiFlash = 15,
    Clockgen = 16,
    RfFrontend = 17,
    Bandwidth = 18,
    TemperatureSensor = 19,
    Rx = 20,
    ExtendedSamplerates = 21,
    ExtendedGain = 22,
}

impl Capability {
    pub const fn bits(self) -> u32 {
        1u32 << (self as u8)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum GainType {
    Lna = 0,
    Rf = 1,
    Mixer = 2,
    Filter = 3,
    Vga = 4,
    Linearity = 5,
    Sensitivity = 6,
    LnaAgc = 7,
    RfAgc = 8,
    MixerAgc = 9,
    FilterAgc = 10,
    Count = 11,
}

pub const GAIN_FLAG_IS_AGC: u8 = 1 << 0;
pub const GAIN_FLAG_IS_PRESET: u8 = 1 << 1;
