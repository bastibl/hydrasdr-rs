//! Internal USB command IDs and bit assignments for HydraSDR RFOne vendor requests.

/// Receiver state values sent with the C `HYDRASDR_VENDOR_REQUEST_RECEIVER_MODE` request.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub(crate) enum ReceiverMode {
    Off = 0,
    Rx = 1,
}

/// USB vendor request numbers copied from `hydrasdr_commands.h`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
#[allow(clippy::enum_variant_names)]
pub(crate) enum VendorRequest {
    ReceiverMode = 1,
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
    SetRfBiasCmd = 20,
    GetSamplerates = 25,
    SetPacking = 26,
    SetRfPort = 28,
    GetCapabilities = 29,
    SetBandwidth = 30,
    GetBandwidths = 31,
    SetGain = 33,
}

/// Capability bit positions reported by the firmware.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub(crate) enum Capability {
    LnaGain = 0,
    MixerGain = 2,
    VgaGain = 4,
    LnaAgc = 5,
    MixerAgc = 7,
    LinearityGain = 9,
    SensitivityGain = 10,
    BiasTee = 11,
    Packing = 12,
    RfPortSelect = 13,
    Bandwidth = 18,
    Rx = 20,
    ExtendedSamplerates = 21,
    ExtendedGain = 22,
}

impl Capability {
    /// Return the single-bit capability mask for this capability.
    pub(crate) const fn bits(self) -> u32 {
        1u32 << (self as u8)
    }
}

/// Gain selector values used by the legacy and extended gain APIs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub(crate) enum GainType {
    Lna = 0,
    Mixer = 2,
    Vga = 4,
    Linearity = 5,
    Sensitivity = 6,
    LnaAgc = 7,
    MixerAgc = 9,
}
