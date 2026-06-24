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
    LowBandwidth,
    HighDefinition,
}

/// Part ID and serial-number words returned by the firmware.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct PartIdSerialNo {
    pub(crate) part_id: [u32; 2],
    pub(crate) serial_no: [u32; 4],
}

/// Internal gain descriptor.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct GainInfo {
    pub(crate) gain_type: crate::commands::GainType,
    pub(crate) min_value: u8,
    pub(crate) max_value: u8,
    pub(crate) step_value: u8,
    pub(crate) default_value: u8,
    pub(crate) value: u8,
    pub(crate) flags: u8,
}

/// Bias tee electrical limits for an RF port.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BiasTeeInfo {
    pub voltage: f32,
    pub max_current_milliamp: f32,
}

/// RF port metadata returned in [`DeviceInfo`].
#[derive(Clone, Debug, PartialEq)]
pub struct RfPortInfo {
    pub name: &'static str,
    pub min_frequency: u64,
    pub max_frequency: u64,
    pub has_bias_tee: bool,
    pub bias_tee: Option<BiasTeeInfo>,
}

/// Device metadata returned by the ergonomic API.
#[derive(Clone, Debug, PartialEq)]
pub struct DeviceInfo {
    pub board_name: &'static str,
    pub firmware_version: String,
    pub serial: Option<u64>,
    pub min_frequency: u64,
    pub max_frequency: u64,
    pub rf_ports: Vec<RfPortInfo>,
    pub current_config: Option<crate::Config>,
}
