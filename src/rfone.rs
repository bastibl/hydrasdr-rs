//! RFOne constants and descriptor helpers used by the high-level driver.

use crate::commands::{Capability, GainType};
use crate::types::{BiasTeeInfo, GainInfo, RfPortInfo};

/// Number of queued streaming transfers used by the C RFOne path.
pub const RFONE_TRANSFER_COUNT: u32 = 16;
/// Bulk-IN endpoint used for RX samples.
pub const RFONE_RX_ENDPOINT: u8 = 0x81;
pub const RFONE_LNA_MAX_GAIN: u8 = 14;
pub const RFONE_MIXER_MAX_GAIN: u8 = 15;
pub const RFONE_VGA_MAX_GAIN: u8 = 15;
pub const RFONE_GAIN_TABLE_SIZE: usize = 22;
pub const RFONE_DEFAULT_GAIN_INDEX: u8 = 10;
pub const RFONE_MIN_FREQ_HZ: u64 = 24_000_000;
pub const RFONE_MAX_FREQ_HZ: u64 = 1_800_000_000;
pub const RFONE_BIAS_TEE_VOLTAGE_V: f32 = 4.5;
pub const RFONE_BIAS_TEE_MAX_MA: f32 = 300.0;

pub const RFONE_LINEARITY_VGA_GAINS: [u8; RFONE_GAIN_TABLE_SIZE] = [
    13, 12, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 8, 7, 6, 5, 4,
];
pub const RFONE_LINEARITY_MIXER_GAINS: [u8; RFONE_GAIN_TABLE_SIZE] = [
    12, 12, 11, 9, 8, 7, 6, 6, 5, 0, 0, 1, 0, 0, 2, 2, 1, 1, 1, 1, 0, 0,
];
pub const RFONE_LINEARITY_LNA_GAINS: [u8; RFONE_GAIN_TABLE_SIZE] = [
    14, 14, 14, 13, 12, 10, 9, 9, 8, 9, 8, 6, 5, 3, 1, 0, 0, 0, 0, 0, 0, 0,
];
pub const RFONE_SENSITIVITY_VGA_GAINS: [u8; RFONE_GAIN_TABLE_SIZE] = [
    13, 12, 11, 10, 9, 8, 7, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4,
];
pub const RFONE_SENSITIVITY_MIXER_GAINS: [u8; RFONE_GAIN_TABLE_SIZE] = [
    12, 12, 12, 12, 11, 10, 10, 9, 9, 8, 7, 4, 4, 4, 3, 2, 2, 1, 0, 0, 0, 0,
];
pub const RFONE_SENSITIVITY_LNA_GAINS: [u8; RFONE_GAIN_TABLE_SIZE] = [
    14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 12, 12, 9, 9, 8, 7, 6, 5, 3, 2, 1, 0,
];

pub const RFONE_HARDCODED_CAPS: u32 = Capability::Rx.bits()
    | Capability::LnaGain.bits()
    | Capability::MixerGain.bits()
    | Capability::VgaGain.bits()
    | Capability::LnaAgc.bits()
    | Capability::MixerAgc.bits()
    | Capability::LinearityGain.bits()
    | Capability::SensitivityGain.bits()
    | Capability::BiasTee.bits()
    | Capability::Packing.bits()
    | Capability::RfPortSelect.bits();

/// Return default gain descriptors for the RFOne direct API.
pub fn default_gain_infos() -> Vec<GainInfo> {
    [
        (GainType::Lna, RFONE_LNA_MAX_GAIN, RFONE_LNA_MAX_GAIN),
        (GainType::Mixer, RFONE_MIXER_MAX_GAIN, RFONE_MIXER_MAX_GAIN),
        (GainType::Vga, RFONE_VGA_MAX_GAIN, RFONE_VGA_MAX_GAIN),
        (
            GainType::Linearity,
            RFONE_GAIN_TABLE_SIZE as u8 - 1,
            RFONE_DEFAULT_GAIN_INDEX,
        ),
        (
            GainType::Sensitivity,
            RFONE_GAIN_TABLE_SIZE as u8 - 1,
            RFONE_DEFAULT_GAIN_INDEX,
        ),
        (GainType::LnaAgc, 1, 0),
        (GainType::MixerAgc, 1, 0),
    ]
    .into_iter()
    .map(|(gain_type, max_value, default_value)| GainInfo {
        gain_type,
        min_value: 0,
        max_value,
        step_value: 1,
        default_value,
        value: default_value,
        flags: 0,
    })
    .collect()
}

/// Return C-parity RF port metadata for RFOne.
pub fn rf_port_infos() -> Vec<RfPortInfo> {
    vec![
        RfPortInfo {
            name: "ANT",
            min_frequency: RFONE_MIN_FREQ_HZ,
            max_frequency: RFONE_MAX_FREQ_HZ,
            has_bias_tee: true,
            bias_tee: Some(BiasTeeInfo {
                voltage: RFONE_BIAS_TEE_VOLTAGE_V,
                max_current_milliamp: RFONE_BIAS_TEE_MAX_MA,
            }),
        },
        RfPortInfo {
            name: "CABLE1",
            min_frequency: RFONE_MIN_FREQ_HZ,
            max_frequency: RFONE_MAX_FREQ_HZ,
            has_bias_tee: false,
            bias_tee: None,
        },
        RfPortInfo {
            name: "CABLE2",
            min_frequency: RFONE_MIN_FREQ_HZ,
            max_frequency: RFONE_MAX_FREQ_HZ,
            has_bias_tee: false,
            bias_tee: None,
        },
    ]
}
