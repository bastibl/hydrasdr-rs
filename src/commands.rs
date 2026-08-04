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
    SetPacking = 26,
    SetRfPort = 28,
}
