//! RFOne constants and descriptor helpers used by the high-level driver.

use crate::commands::Capability;
use crate::types::{BiasTeeInfo, RfPortInfo};

/// Number of queued streaming transfers used by the C RFOne path.
pub(crate) const RFONE_TRANSFER_COUNT: u32 = 16;
/// Bulk-IN endpoint used for RX samples.
pub(crate) const RFONE_RX_ENDPOINT: u8 = 0x81;
pub(crate) const RFONE_LNA_MAX_GAIN: u8 = 14;
pub(crate) const RFONE_MIXER_MAX_GAIN: u8 = 15;
pub(crate) const RFONE_VGA_MAX_GAIN: u8 = 15;
pub(crate) const RFONE_GAIN_TABLE_SIZE: usize = 22;
pub(crate) const RFONE_MIN_FREQ_HZ: u64 = 24_000_000;
pub(crate) const RFONE_MAX_FREQ_HZ: u64 = 1_800_000_000;
pub(crate) const RFONE_BIAS_TEE_VOLTAGE_V: f32 = 4.5;
pub(crate) const RFONE_BIAS_TEE_MAX_MA: f32 = 300.0;

pub(crate) const RFONE_LINEARITY_VGA_GAINS: [u8; RFONE_GAIN_TABLE_SIZE] = [
    13, 12, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 8, 7, 6, 5, 4,
];
pub(crate) const RFONE_LINEARITY_MIXER_GAINS: [u8; RFONE_GAIN_TABLE_SIZE] = [
    12, 12, 11, 9, 8, 7, 6, 6, 5, 0, 0, 1, 0, 0, 2, 2, 1, 1, 1, 1, 0, 0,
];
pub(crate) const RFONE_LINEARITY_LNA_GAINS: [u8; RFONE_GAIN_TABLE_SIZE] = [
    14, 14, 14, 13, 12, 10, 9, 9, 8, 9, 8, 6, 5, 3, 1, 0, 0, 0, 0, 0, 0, 0,
];
pub(crate) const RFONE_SENSITIVITY_VGA_GAINS: [u8; RFONE_GAIN_TABLE_SIZE] = [
    13, 12, 11, 10, 9, 8, 7, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4,
];
pub(crate) const RFONE_SENSITIVITY_MIXER_GAINS: [u8; RFONE_GAIN_TABLE_SIZE] = [
    12, 12, 12, 12, 11, 10, 10, 9, 9, 8, 7, 4, 4, 4, 3, 2, 2, 1, 0, 0, 0, 0,
];
pub(crate) const RFONE_SENSITIVITY_LNA_GAINS: [u8; RFONE_GAIN_TABLE_SIZE] = [
    14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 12, 12, 9, 9, 8, 7, 6, 5, 3, 2, 1, 0,
];

pub(crate) const RFONE_HARDCODED_CAPS: u32 = Capability::Rx.bits()
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

/// Return C-parity RF port metadata for RFOne.
pub(crate) fn rf_port_infos() -> Vec<RfPortInfo> {
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
