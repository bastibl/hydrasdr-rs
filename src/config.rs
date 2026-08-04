//! Receiver configuration.

use crate::errors::{Error, Result};
use crate::rfone::{
    RFONE_LNA_MAX_GAIN, RFONE_MAX_FREQ_HZ, RFONE_MIN_FREQ_HZ, RFONE_MIXER_MAX_GAIN,
    RFONE_VGA_MAX_GAIN,
};
use crate::types::{DecimationMode, SampleType};

const DEFAULT_FREQUENCY_HZ: u64 = 100_000_000;
const DEFAULT_SAMPLE_RATE_HZ: u32 = 10_000_000;
const MIN_SAMPLE_RATE_HZ: u32 = 10_000;
const MIN_BANDWIDTH_HZ: u32 = 1_000;
const MAX_PRESET_GAIN: u8 = 21;
const MAX_VENDOR_INDEX_OR_KHZ: u32 = u16::MAX as u32;
const MAX_RAW_SAMPLE_RATE_HZ: u32 = MAX_VENDOR_INDEX_OR_KHZ * 1_000 + 999;
const MAX_F32_IQ_SAMPLE_RATE_HZ: u32 = (((MAX_VENDOR_INDEX_OR_KHZ + 1) * 1_000) - 1) / 2;
const MAX_MANUAL_BANDWIDTH_HZ: u32 = MAX_VENDOR_INDEX_OR_KHZ * 1_000 + 999;

/// Device selection used by [`crate::DeviceBuilder`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum DeviceSelector {
    /// Open the first visible HydraSDR RFOne.
    First,
    /// Open the HydraSDR RFOne with a parsed 64-bit serial number.
    Serial(u64),
}

/// Analog bandwidth policy.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Bandwidth {
    /// Leave bandwidth selection to firmware defaults.
    Auto,
    /// Set an explicit bandwidth in Hz before setting the sample rate.
    ///
    /// Manual bandwidths must be in the inclusive range `1_000..=65_535_999`.
    ManualHz(u32),
}

/// RF input port selector.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum RfPort {
    /// First RF input port.
    Rx0 = 0,
    /// Second RF input port.
    Rx1 = 1,
    /// Third RF input port.
    Rx2 = 2,
}

/// High-level sample format names.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SampleFormat {
    /// Raw ADC USB blocks at the selected firmware/hardware sample rate.
    ///
    /// Raw ADC sample rates must be in the inclusive range `10_000..=65_535_999` Hz.
    RawAdc,
    /// Converted 32-bit float IQ samples with optional host-side decimation.
    ///
    /// Float IQ sample rates must be in the inclusive range `10_000..=32_767_999` Hz.
    F32Iq,
}

impl SampleFormat {
    pub(crate) const fn sample_type(self) -> SampleType {
        match self {
            Self::RawAdc => SampleType::Raw,
            Self::F32Iq => SampleType::Float32Iq,
        }
    }
}

/// Preset gain plans for common RFOne operation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GainPreset {
    /// Apply the RFOne linearity preset table.
    Linearity(u8),
    /// Apply the RFOne sensitivity preset table.
    Sensitivity(u8),
}

/// Gain configuration applied after sample/rate/RF-port setup.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GainConfig {
    /// Leave direct gain state unchanged.
    ///
    /// This is opt-in. The standard [`Config`] default explicitly sets every
    /// gain stage to 0 dB and disables both AGCs.
    Unchanged,
    /// Apply one RFOne preset.
    Preset(GainPreset),
    /// Apply explicit component gains/AGC bits where present.
    Manual {
        /// LNA gain value, if explicitly configured.
        ///
        /// RFOne accepts values in the inclusive range `0..=14`.
        lna: Option<u8>,
        /// Mixer gain value, if explicitly configured.
        ///
        /// RFOne accepts values in the inclusive range `0..=15`.
        mixer: Option<u8>,
        /// VGA gain value, if explicitly configured.
        ///
        /// RFOne accepts values in the inclusive range `0..=15`.
        vga: Option<u8>,
        /// LNA AGC enable state, if explicitly configured.
        lna_agc: Option<bool>,
        /// Mixer AGC enable state, if explicitly configured.
        mixer_agc: Option<bool>,
    },
}

/// Return the complete standard manual-gain configuration.
impl Default for GainConfig {
    fn default() -> Self {
        Self::Manual {
            lna: Some(0),
            mixer: Some(0),
            vga: Some(0),
            lna_agc: Some(false),
            mixer_agc: Some(false),
        }
    }
}

impl From<GainPreset> for GainConfig {
    fn from(value: GainPreset) -> Self {
        Self::Preset(value)
    }
}

/// Reusable high-level receiver configuration.
///
/// The default is complete: it selects RX0, disables the bias tee and both
/// AGCs, and sets the LNA, mixer, and VGA gains to 0 dB. Callers can opt out of
/// gain writes with [`GainConfig::Unchanged`] or construct partial manual gain
/// updates explicitly.
///
/// Building a config validates ranges without opening USB hardware, so this is
/// safe in doctests and CI:
///
/// ```
/// use hydrasdr_rs::{Bandwidth, Config, GainPreset, SampleFormat};
///
/// let config = Config::builder()
///     .frequency_hz(144_500_000)
///     .sample_rate_hz(10_000_000)
///     .bandwidth(Bandwidth::Auto)
///     .sample_format(SampleFormat::RawAdc)
///     .gain(GainPreset::Linearity(10))
///     .build()?;
///
/// assert_eq!(config.frequency_hz(), 144_500_000);
/// assert_eq!(config.sample_format(), SampleFormat::RawAdc);
/// # Ok::<(), hydrasdr_rs::Error>(())
/// ```
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Config {
    frequency_hz: u64,
    sample_rate_hz: u32,
    bandwidth: Bandwidth,
    sample_format: SampleFormat,
    decimation_mode: DecimationMode,
    rf_port: Option<RfPort>,
    gain: GainConfig,
    bias_tee: Option<bool>,
    packing: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            frequency_hz: DEFAULT_FREQUENCY_HZ,
            sample_rate_hz: DEFAULT_SAMPLE_RATE_HZ,
            bandwidth: Bandwidth::Auto,
            sample_format: SampleFormat::RawAdc,
            decimation_mode: DecimationMode::LowBandwidth,
            rf_port: Some(RfPort::Rx0),
            gain: GainConfig::default(),
            bias_tee: Some(false),
            packing: false,
        }
    }
}

impl Config {
    /// Start building a high-level receiver configuration.
    ///
    /// ```
    /// use hydrasdr_rs::{Config, GainPreset};
    ///
    /// let config = Config::builder()
    ///     .frequency_hz(100_000_000)
    ///     .sample_rate_hz(10_000_000)
    ///     .gain(GainPreset::Sensitivity(6))
    ///     .build()?;
    ///
    /// assert_eq!(config.sample_rate_hz(), 10_000_000);
    /// # Ok::<(), hydrasdr_rs::Error>(())
    /// ```
    pub fn builder() -> ConfigBuilder {
        ConfigBuilder::default()
    }

    /// Tuned center frequency in Hz.
    pub const fn frequency_hz(&self) -> u64 {
        self.frequency_hz
    }

    /// ADC/sample rate in Hz.
    pub const fn sample_rate_hz(&self) -> u32 {
        self.sample_rate_hz
    }

    /// Configured bandwidth policy.
    pub const fn bandwidth(&self) -> Bandwidth {
        self.bandwidth
    }

    /// Configured high-level sample format.
    pub const fn sample_format(&self) -> SampleFormat {
        self.sample_format
    }

    /// Virtual IQ-rate hardware/host decimation policy.
    pub const fn decimation_mode(&self) -> DecimationMode {
        self.decimation_mode
    }

    /// Configured RF port, if explicitly selected.
    pub const fn rf_port(&self) -> Option<RfPort> {
        self.rf_port
    }

    /// Configured gain plan.
    pub const fn gain(&self) -> GainConfig {
        self.gain
    }

    /// Configured bias tee state, if explicitly selected.
    pub const fn bias_tee(&self) -> Option<bool> {
        self.bias_tee
    }

    /// Whether packed samples should be requested.
    pub const fn packing(&self) -> bool {
        self.packing
    }

    /// Validate this configuration without touching USB.
    pub fn validate(&self) -> Result<()> {
        validate_frequency(self.frequency_hz)?;
        validate_sample_rate(self.sample_rate_hz, self.sample_format)?;
        validate_bandwidth(self.bandwidth)?;
        validate_gain(self.gain)?;
        validate_format_decimation(self.sample_format, self.decimation_mode)?;
        validate_format_packing(self.sample_format, self.packing)?;
        Ok(())
    }
}

/// Builder for [`Config`].
///
/// ```
/// use hydrasdr_rs::{Config, SampleFormat};
///
/// let config = Config::builder()
///     .frequency_hz(915_000_000)
///     .sample_rate_hz(2_000_000)
///     .bandwidth_hz(1_750_000)
///     .sample_format(SampleFormat::RawAdc)
///     .packing(true)
///     .build()?;
///
/// assert_eq!(config.bandwidth(), hydrasdr_rs::Bandwidth::ManualHz(1_750_000));
/// assert_eq!(config.sample_format(), SampleFormat::RawAdc);
/// assert!(config.packing());
/// # Ok::<(), hydrasdr_rs::Error>(())
/// ```
#[derive(Clone, Debug, Default)]
pub struct ConfigBuilder {
    config: Config,
}

impl ConfigBuilder {
    /// Set the tuned center frequency in Hz.
    ///
    /// RFOne accepts center frequencies in the inclusive range
    /// `24_000_000..=1_800_000_000` Hz.
    pub fn frequency_hz(mut self, value: u64) -> Self {
        self.config.frequency_hz = value;
        self
    }

    /// Set the ADC/sample rate in Hz.
    ///
    /// [`SampleFormat::RawAdc`] accepts `10_000..=65_535_999` Hz.
    /// [`SampleFormat::F32Iq`] accepts `10_000..=32_767_999` Hz because
    /// the hardware rate is doubled before host-side IQ conversion.
    pub fn sample_rate_hz(mut self, value: u32) -> Self {
        self.config.sample_rate_hz = value;
        self
    }

    /// Set the analog bandwidth policy.
    ///
    /// [`Bandwidth::ManualHz`] values must be in the inclusive range
    /// `1_000..=65_535_999` Hz.
    pub fn bandwidth(mut self, value: Bandwidth) -> Self {
        self.config.bandwidth = value;
        self
    }

    /// Set an explicit analog bandwidth in Hz.
    ///
    /// This is shorthand for [`ConfigBuilder::bandwidth`] with
    /// [`Bandwidth::ManualHz`]. Values must be in the inclusive range
    /// `1_000..=65_535_999` Hz.
    pub fn bandwidth_hz(self, value: u32) -> Self {
        self.bandwidth(Bandwidth::ManualHz(value))
    }

    /// Set the high-level sample format.
    ///
    /// The sample format determines which sample-rate range is valid.
    pub fn sample_format(mut self, value: SampleFormat) -> Self {
        self.config.sample_format = value;
        self
    }

    /// Set the firmware/host decimation policy for float IQ samples.
    ///
    /// Decimation is only valid with [`SampleFormat::F32Iq`].
    pub fn decimation_mode(mut self, value: DecimationMode) -> Self {
        self.config.decimation_mode = value;
        self
    }

    /// Select the RF input port.
    pub fn rf_port(mut self, value: RfPort) -> Self {
        self.config.rf_port = Some(value);
        self
    }

    /// Set the gain configuration.
    ///
    /// Preset gain indexes must be in the inclusive range `0..=21`.
    /// Manual component gains use the ranges documented on [`GainConfig::Manual`].
    pub fn gain(mut self, value: impl Into<GainConfig>) -> Self {
        self.config.gain = value.into();
        self
    }

    /// Enable or disable the RF port bias tee.
    pub fn bias_tee(mut self, enabled: bool) -> Self {
        self.config.bias_tee = Some(enabled);
        self
    }

    /// Enable or disable packed raw-sample transfers.
    ///
    /// Packing is only valid with [`SampleFormat::RawAdc`].
    pub fn packing(mut self, enabled: bool) -> Self {
        self.config.packing = enabled;
        self
    }

    /// Validate and build a reusable configuration.
    pub fn build(self) -> Result<Config> {
        self.config.validate()?;
        Ok(self.config)
    }
}

pub(crate) fn validate_frequency(value: u64) -> Result<()> {
    if !(RFONE_MIN_FREQ_HZ..=RFONE_MAX_FREQ_HZ).contains(&value) {
        return Err(Error::invalid_config(
            "frequency_hz",
            "must be in 24_000_000..=1_800_000_000 Hz",
        ));
    }
    Ok(())
}

pub(crate) fn validate_sample_rate(value: u32, sample_format: SampleFormat) -> Result<()> {
    if value < MIN_SAMPLE_RATE_HZ {
        return Err(Error::invalid_config(
            "sample_rate_hz",
            "must be at least 10_000 Hz",
        ));
    }
    let max_hz = match sample_format {
        SampleFormat::RawAdc => MAX_RAW_SAMPLE_RATE_HZ,
        SampleFormat::F32Iq => MAX_F32_IQ_SAMPLE_RATE_HZ,
    };
    if value > max_hz {
        return Err(Error::invalid_config(
            "sample_rate_hz",
            "must fit the HydraSDR vendor request parameter",
        ));
    }
    Ok(())
}

fn validate_format_decimation(
    sample_format: SampleFormat,
    decimation_mode: DecimationMode,
) -> Result<()> {
    if sample_format == SampleFormat::RawAdc && decimation_mode == DecimationMode::HighDefinition {
        return Err(Error::invalid_config(
            "decimation_mode",
            "HighDefinition is only valid for converted F32Iq streams",
        ));
    }
    Ok(())
}

fn validate_format_packing(sample_format: SampleFormat, packing: bool) -> Result<()> {
    if sample_format == SampleFormat::F32Iq && packing {
        return Err(Error::invalid_config(
            "packing",
            "packed samples are only valid for raw ADC streams",
        ));
    }
    Ok(())
}

pub(crate) fn validate_bandwidth(value: Bandwidth) -> Result<()> {
    if let Bandwidth::ManualHz(hz) = value
        && hz < MIN_BANDWIDTH_HZ
    {
        return Err(Error::invalid_config(
            "bandwidth_hz",
            "manual bandwidth must be at least 1_000 Hz",
        ));
    }
    if let Bandwidth::ManualHz(hz) = value
        && hz > MAX_MANUAL_BANDWIDTH_HZ
    {
        return Err(Error::invalid_config(
            "bandwidth_hz",
            "manual bandwidth must fit the HydraSDR vendor request parameter",
        ));
    }
    Ok(())
}

pub(crate) fn validate_gain(gain: GainConfig) -> Result<()> {
    match gain {
        GainConfig::Preset(GainPreset::Linearity(value) | GainPreset::Sensitivity(value))
            if value > MAX_PRESET_GAIN =>
        {
            Err(Error::invalid_config(
                "gain",
                "RFOne preset gain must be in 0..=21",
            ))
        }
        GainConfig::Manual {
            lna: Some(value), ..
        } if value > RFONE_LNA_MAX_GAIN => Err(Error::invalid_config(
            "gain",
            "manual LNA gain must be in 0..=14",
        )),
        GainConfig::Manual {
            mixer: Some(value), ..
        } if value > RFONE_MIXER_MAX_GAIN => Err(Error::invalid_config(
            "gain",
            "manual mixer gain must be in 0..=15",
        )),
        GainConfig::Manual {
            vga: Some(value), ..
        } if value > RFONE_VGA_MAX_GAIN => Err(Error::invalid_config(
            "gain",
            "manual VGA gain must be in 0..=15",
        )),
        _ => Ok(()),
    }
}
