use crate::commands::Capability;
use crate::constants::{DEFAULT_BUFFER_SIZE, PACKED_BUFFER_SIZE};
use crate::types::{SampleType, sample_type_bit};

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

pub const RFONE_TRANSFER_COUNT: u32 = 16;
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
