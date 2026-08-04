//! Public data structures returned by the HydraSDR API.

/// HydraSDR board identifier values mirrored from the C driver.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub(crate) enum BoardId {
    ProtoHydraSdr = 0,
    HydraSdrRfOneOfficial = 1,
    Invalid = 0xff,
}

/// Raw board ID value that is not known to this direct translation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct UnknownBoardId(pub(crate) u8);

impl TryFrom<u8> for BoardId {
    type Error = UnknownBoardId;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::ProtoHydraSdr),
            1 => Ok(Self::HydraSdrRfOneOfficial),
            0xff => Ok(Self::Invalid),
            other => Err(UnknownBoardId(other)),
        }
    }
}

/// Sample type values accepted by the direct API.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub(crate) enum SampleType {
    Float32Iq = 0,
    Raw = 5,
}

/// Host-side IQ decimation mode.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DecimationMode {
    /// Prefer lower host bandwidth by using firmware decimation by 2.
    LowBandwidth,
    /// Prefer higher-definition IQ conversion without firmware decimation.
    HighDefinition,
}

/// Part ID and serial-number words returned by the firmware.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct PartIdSerialNo {
    pub(crate) part_id: [u32; 2],
    pub(crate) serial_no: [u32; 4],
}

/// Bias tee electrical limits for an RF port.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BiasTeeInfo {
    /// Bias tee voltage in volts.
    pub voltage: f32,
    /// Maximum bias tee current in milliamps.
    pub max_current_milliamp: f32,
}

/// RF port metadata returned in [`DeviceInfo`].
#[derive(Clone, Debug, PartialEq)]
pub struct RfPortInfo {
    /// Firmware-provided RF port name.
    pub name: &'static str,
    /// Minimum tunable frequency for this port, in Hz.
    pub min_frequency: u64,
    /// Maximum tunable frequency for this port, in Hz.
    pub max_frequency: u64,
    /// Whether this port exposes a bias tee.
    pub has_bias_tee: bool,
    /// Bias tee electrical limits, if this port has a bias tee.
    pub bias_tee: Option<BiasTeeInfo>,
}

/// Device metadata returned by the ergonomic API.
#[derive(Clone, Debug, PartialEq)]
pub struct DeviceInfo {
    /// Board name detected from the firmware board ID.
    pub board_name: &'static str,
    /// Firmware version string.
    pub firmware_version: String,
    /// Parsed 64-bit serial number, if available.
    pub serial: Option<u64>,
    /// Minimum tunable device frequency in Hz.
    pub min_frequency: u64,
    /// Maximum tunable device frequency in Hz.
    pub max_frequency: u64,
    /// RF ports reported for this device.
    pub rf_ports: Vec<RfPortInfo>,
    /// Shared state of settings applied through this crate.
    pub active_state: crate::ActiveState,
}
