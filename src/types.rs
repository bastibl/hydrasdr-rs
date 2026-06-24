//! Public data structures returned by the HydraSDR API.

#![allow(dead_code)]

/// HydraSDR board identifier values mirrored from the C driver.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum BoardId {
    ProtoHydraSdr = 0,
    HydraSdrRfOneOfficial = 1,
    Invalid = 0xff,
}

/// Raw board ID value that is not known to this direct translation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct UnknownBoardId(pub u8);

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

/// Return the C-style human-readable board name for a raw board ID byte.
pub fn board_id_name_raw(board_id: u8) -> &'static str {
    match BoardId::try_from(board_id) {
        Ok(BoardId::Invalid) => "Invalid Board ID",
        Ok(BoardId::ProtoHydraSdr) => "HydraSDR RFOne Legacy VID/PID",
        Ok(BoardId::HydraSdrRfOneOfficial) => "HydraSDR RFOne Official VID/PID",
        Err(_) => "Unknown Board ID",
    }
}

/// Sample type values accepted by the direct API.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum SampleType {
    Float32Iq = 0,
    Float32Real = 1,
    Int16Iq = 2,
    Int16Real = 3,
    Uint16Real = 4,
    Raw = 5,
    Int8Iq = 6,
    Uint8Iq = 7,
    Int8Real = 8,
    Uint8Real = 9,
    End = 10,
}

/// Return the C capability bit for a sample type.
pub const fn sample_type_bit(sample_type: SampleType) -> u16 {
    1u16 << (sample_type as u8)
}

/// Host-side IQ decimation mode.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DecimationMode {
    LowBandwidth,
    HighDefinition,
}

/// Library version tuple mirroring the C struct layout.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LibVersion {
    pub major_version: u32,
    pub minor_version: u32,
    pub revision: u32,
}

/// Part ID and serial-number words returned by the firmware.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PartIdSerialNo {
    pub part_id: [u32; 2],
    pub serial_no: [u32; 4],
}

/// Internal gain descriptor.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct GainInfo {
    pub gain_type: crate::commands::GainType,
    pub min_value: u8,
    pub max_value: u8,
    pub step_value: u8,
    pub default_value: u8,
    pub value: u8,
    pub flags: u8,
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

/// Hardware component metadata returned in [`DeviceInfo`].
#[derive(Clone, Debug, PartialEq)]
pub struct ComponentInfo {
    pub name: &'static str,
    pub register_count: u32,
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

/// Temperature reading shape reserved for future hardware support.
///
/// RFOne temperature querying is currently reported as unsupported.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Temperature {
    pub valid: bool,
    pub temperature_celsius: f32,
    pub temperature_fahrenheit: f32,
}
