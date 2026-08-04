//! Public data structures returned by the HydraSDR driver.

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
    /// Prefer the lowest firmware rate that can produce the requested effective
    /// IQ rate, reducing USB and host processing bandwidth.
    LowBandwidth,
    /// Prefer the highest firmware rate that can produce the requested
    /// effective IQ rate, then decimate on the host for greater oversampling.
    HighDefinition,
}

/// Part ID and serial-number words returned by the firmware.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct PartIdSerialNo {
    pub(crate) part_id: [u32; 2],
    pub(crate) serial_no: [u32; 4],
}

/// RF port metadata returned in [`DeviceInfo`].
#[derive(Clone, Debug, PartialEq)]
pub struct RfPortInfo {
    /// Protocol selector used to activate this port.
    pub port: crate::RfPort,
    /// Firmware-provided RF port name.
    pub name: &'static str,
    /// Minimum tunable frequency for this port, in Hz.
    pub min_frequency: u64,
    /// Maximum tunable frequency for this port, in Hz.
    pub max_frequency: u64,
    /// Whether this port exposes a bias tee.
    pub has_bias_tee: bool,
}

/// Immutable device metadata reported while opening the device.
#[derive(Clone, Debug, PartialEq)]
pub struct DeviceInfo {
    /// Board name detected from the firmware board ID.
    pub board_name: &'static str,
    /// Firmware version string.
    pub firmware_version: String,
    /// Parsed 64-bit serial number, if available.
    pub serial: Option<u64>,
    /// RF ports reported for this device.
    pub rf_ports: Vec<RfPortInfo>,
}
