//! RFOne constants and descriptor helpers copied from the C reference driver.

#![allow(dead_code)]

use crate::commands::{Capability, GainType};
use crate::constants::{DEFAULT_BUFFER_SIZE, PACKED_BUFFER_SIZE};
use crate::types::{BiasTeeInfo, ComponentInfo, GainInfo, RfPortInfo, SampleType, sample_type_bit};

/// Static RFOne hardware facts used by parity tests and direct device info.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RfOneSpec {
    pub transfer_count: u32,
    pub default_buffer_size: usize,
    pub packed_buffer_size: usize,
    pub bulk_rx_endpoint: u8,
    pub lna_max_gain: u8,
    pub mixer_max_gain: u8,
    pub vga_max_gain: u8,
    pub gain_table_size: usize,
    pub default_gain_index: u8,
    pub min_frequency_hz: u64,
    pub max_frequency_hz: u64,
    pub typical_power_mw: f32,
    pub max_power_mw: f32,
    pub max_safe_temp_c: f32,
    pub bias_tee_voltage_v: f32,
    pub bias_tee_max_ma: f32,
    pub rf_port_count: u8,
    pub gpio_count: u16,
    pub component_count: u8,
    pub rf_frontend_registers: u32,
    pub clockgen_registers: u32,
}

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
pub const RFONE_TYPICAL_POWER_MW: f32 = 1_800.0;
pub const RFONE_MAX_POWER_MW: f32 = 3_200.0;
pub const RFONE_MAX_SAFE_TEMP_C: f32 = 70.0;
pub const RFONE_BIAS_TEE_VOLTAGE_V: f32 = 4.5;
pub const RFONE_BIAS_TEE_MAX_MA: f32 = 300.0;
pub const RFONE_RF_PORT_COUNT: u8 = 3;
pub const RFONE_GPIO_COUNT: u16 = 18;
pub const RFONE_COMPONENT_COUNT: u8 = 2;
pub const RFONE_RF_FRONTEND_REGS: u32 = 32;
pub const RFONE_CLOCKGEN_REGS: u32 = 256;

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

pub const RFONE_SAMPLE_TYPES: u16 = sample_type_bit(SampleType::Float32Iq)
    | sample_type_bit(SampleType::Float32Real)
    | sample_type_bit(SampleType::Int16Iq)
    | sample_type_bit(SampleType::Int16Real)
    | sample_type_bit(SampleType::Uint16Real)
    | sample_type_bit(SampleType::Raw);

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
    | Capability::RfPortSelect.bits()
    | Capability::Gpio.bits()
    | Capability::SpiFlash.bits()
    | Capability::Clockgen.bits()
    | Capability::RfFrontend.bits();

/// Complete RFOne static specification aggregate.
pub const RFONE_SPEC: RfOneSpec = RfOneSpec {
    transfer_count: RFONE_TRANSFER_COUNT,
    default_buffer_size: DEFAULT_BUFFER_SIZE,
    packed_buffer_size: PACKED_BUFFER_SIZE,
    bulk_rx_endpoint: RFONE_RX_ENDPOINT,
    lna_max_gain: RFONE_LNA_MAX_GAIN,
    mixer_max_gain: RFONE_MIXER_MAX_GAIN,
    vga_max_gain: RFONE_VGA_MAX_GAIN,
    gain_table_size: RFONE_GAIN_TABLE_SIZE,
    default_gain_index: RFONE_DEFAULT_GAIN_INDEX,
    min_frequency_hz: RFONE_MIN_FREQ_HZ,
    max_frequency_hz: RFONE_MAX_FREQ_HZ,
    typical_power_mw: RFONE_TYPICAL_POWER_MW,
    max_power_mw: RFONE_MAX_POWER_MW,
    max_safe_temp_c: RFONE_MAX_SAFE_TEMP_C,
    bias_tee_voltage_v: RFONE_BIAS_TEE_VOLTAGE_V,
    bias_tee_max_ma: RFONE_BIAS_TEE_MAX_MA,
    rf_port_count: RFONE_RF_PORT_COUNT,
    gpio_count: RFONE_GPIO_COUNT,
    component_count: RFONE_COMPONENT_COUNT,
    rf_frontend_registers: RFONE_RF_FRONTEND_REGS,
    clockgen_registers: RFONE_CLOCKGEN_REGS,
};

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

/// Return C-parity component metadata for RFOne.
pub fn component_infos() -> Vec<ComponentInfo> {
    vec![
        ComponentInfo {
            name: "RafaelMicro R828D",
            register_count: RFONE_RF_FRONTEND_REGS,
        },
        ComponentInfo {
            name: "Skyworks SI5351C",
            register_count: RFONE_CLOCKGEN_REGS,
        },
    ]
}
