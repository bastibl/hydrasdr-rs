//! Receiver configuration.

use crate::commands::GainType;
use crate::device::HydraSdr;
use crate::errors::{Error, Result};
use crate::rfone::{RFONE_MAX_FREQ_HZ, RFONE_MIN_FREQ_HZ};
use crate::types::{DecimationMode, SampleType};
use crate::usb::control::{AsyncControlBackend, ControlBackend};

const DEFAULT_FREQUENCY_HZ: u64 = 100_000_000;
const DEFAULT_SAMPLE_RATE_HZ: u32 = 10_000_000;
const MIN_SAMPLE_RATE_HZ: u32 = 10_000;
const MIN_BANDWIDTH_HZ: u32 = 1_000;
const MAX_PRESET_GAIN: u8 = 21;

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
    ManualHz(u32),
}

/// RF input port selector.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum RfPort {
    Rx0 = 0,
    Rx1 = 1,
    Rx2 = 2,
}

/// High-level sample format names.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SampleFormat {
    /// Raw ADC USB blocks at the selected firmware/hardware sample rate.
    RawAdc,
    /// Converted 32-bit float IQ samples with optional host-side decimation.
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
    Unchanged,
    /// Apply one RFOne preset.
    Preset(GainPreset),
    /// Apply explicit component gains/AGC bits where present.
    Manual {
        lna: Option<u8>,
        mixer: Option<u8>,
        vga: Option<u8>,
        lna_agc: Option<bool>,
        mixer_agc: Option<bool>,
    },
}

impl From<GainPreset> for GainConfig {
    fn from(value: GainPreset) -> Self {
        Self::Preset(value)
    }
}

/// Reusable high-level receiver configuration.
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
            rf_port: None,
            gain: GainConfig::Unchanged,
            bias_tee: None,
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
        validate_sample_rate(self.sample_rate_hz)?;
        validate_bandwidth(self.bandwidth)?;
        validate_gain(self.gain)?;
        validate_format_decimation(self.sample_format, self.decimation_mode)?;
        validate_format_packing(self.sample_format, self.packing)?;
        Ok(())
    }

    pub(crate) fn apply_direct<C>(&self, direct: &mut HydraSdr<C>) -> Result<()>
    where
        C: ControlBackend,
    {
        self.validate()?;
        direct.set_freq(self.frequency_hz)?;
        direct.set_sample_type(self.sample_format.sample_type())?;
        direct.set_decimation_mode(self.decimation_mode)?;
        if let Bandwidth::ManualHz(bandwidth_hz) = self.bandwidth {
            direct.set_bandwidth(bandwidth_hz)?;
        }
        direct.set_samplerate(self.sample_rate_hz)?;
        if let Some(port) = self.rf_port {
            direct.set_rf_port(port)?;
        }
        apply_gain_direct(direct, self.gain)?;
        if let Some(enabled) = self.bias_tee {
            direct.set_rf_bias(u8::from(enabled))?;
        }
        direct.set_packing(u8::from(self.packing))?;
        Ok(())
    }

    pub(crate) async fn apply_direct_async<C>(&self, direct: &mut HydraSdr<C>) -> Result<()>
    where
        C: AsyncControlBackend + ControlBackend,
    {
        self.validate()?;
        direct.set_freq_async(self.frequency_hz).await?;
        direct.set_sample_type(self.sample_format.sample_type())?;
        direct
            .set_decimation_mode_async(self.decimation_mode)
            .await?;
        if let Bandwidth::ManualHz(bandwidth_hz) = self.bandwidth {
            direct.set_bandwidth_async(bandwidth_hz).await?;
        }
        direct.set_samplerate_async(self.sample_rate_hz).await?;
        if let Some(port) = self.rf_port {
            direct.set_rf_port_async(port).await?;
        }
        apply_gain_direct_async(direct, self.gain).await?;
        if let Some(enabled) = self.bias_tee {
            direct.set_rf_bias_async(u8::from(enabled)).await?;
        }
        direct.set_packing_async(u8::from(self.packing)).await?;
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
    pub fn frequency_hz(mut self, value: u64) -> Self {
        self.config.frequency_hz = value;
        self
    }

    pub fn sample_rate_hz(mut self, value: u32) -> Self {
        self.config.sample_rate_hz = value;
        self
    }

    pub fn bandwidth(mut self, value: Bandwidth) -> Self {
        self.config.bandwidth = value;
        self
    }

    pub fn bandwidth_hz(self, value: u32) -> Self {
        self.bandwidth(Bandwidth::ManualHz(value))
    }

    pub fn sample_format(mut self, value: SampleFormat) -> Self {
        self.config.sample_format = value;
        self
    }

    pub fn decimation_mode(mut self, value: DecimationMode) -> Self {
        self.config.decimation_mode = value;
        self
    }

    pub fn rf_port(mut self, value: RfPort) -> Self {
        self.config.rf_port = Some(value);
        self
    }

    pub fn gain(mut self, value: impl Into<GainConfig>) -> Self {
        self.config.gain = value.into();
        self
    }

    pub fn bias_tee(mut self, enabled: bool) -> Self {
        self.config.bias_tee = Some(enabled);
        self
    }

    pub fn packing(mut self, enabled: bool) -> Self {
        self.config.packing = enabled;
        self
    }

    pub fn build(self) -> Result<Config> {
        self.config.validate()?;
        Ok(self.config)
    }
}

fn apply_gain_direct<C>(direct: &mut HydraSdr<C>, gain: GainConfig) -> Result<()>
where
    C: ControlBackend,
{
    match gain {
        GainConfig::Unchanged => Ok(()),
        GainConfig::Preset(GainPreset::Linearity(value)) => {
            direct.set_gain(GainType::Linearity, value)
        }
        GainConfig::Preset(GainPreset::Sensitivity(value)) => {
            direct.set_gain(GainType::Sensitivity, value)
        }
        GainConfig::Manual {
            lna,
            mixer,
            vga,
            lna_agc,
            mixer_agc,
        } => {
            if let Some(value) = lna {
                direct.set_lna_gain(value)?;
            }
            if let Some(value) = mixer {
                direct.set_mixer_gain(value)?;
            }
            if let Some(value) = vga {
                direct.set_vga_gain(value)?;
            }
            if let Some(enabled) = lna_agc {
                direct.set_lna_agc(u8::from(enabled))?;
            }
            if let Some(enabled) = mixer_agc {
                direct.set_mixer_agc(u8::from(enabled))?;
            }
            Ok(())
        }
    }
}

async fn apply_gain_direct_async<C>(direct: &mut HydraSdr<C>, gain: GainConfig) -> Result<()>
where
    C: AsyncControlBackend + ControlBackend,
{
    match gain {
        GainConfig::Unchanged => Ok(()),
        GainConfig::Preset(GainPreset::Linearity(value)) => {
            direct.set_gain_async(GainType::Linearity, value).await
        }
        GainConfig::Preset(GainPreset::Sensitivity(value)) => {
            direct.set_gain_async(GainType::Sensitivity, value).await
        }
        GainConfig::Manual {
            lna,
            mixer,
            vga,
            lna_agc,
            mixer_agc,
        } => {
            if let Some(value) = lna {
                direct.set_lna_gain_async(value).await?;
            }
            if let Some(value) = mixer {
                direct.set_mixer_gain_async(value).await?;
            }
            if let Some(value) = vga {
                direct.set_vga_gain_async(value).await?;
            }
            if let Some(enabled) = lna_agc {
                direct.set_lna_agc_async(u8::from(enabled)).await?;
            }
            if let Some(enabled) = mixer_agc {
                direct.set_mixer_agc_async(u8::from(enabled)).await?;
            }
            Ok(())
        }
    }
}

fn validate_frequency(value: u64) -> Result<()> {
    if !(RFONE_MIN_FREQ_HZ..=RFONE_MAX_FREQ_HZ).contains(&value) {
        return Err(Error::invalid_config(
            "frequency_hz",
            "must be in 24_000_000..=1_800_000_000 Hz",
        ));
    }
    Ok(())
}

fn validate_sample_rate(value: u32) -> Result<()> {
    if value < MIN_SAMPLE_RATE_HZ {
        return Err(Error::invalid_config(
            "sample_rate_hz",
            "must be at least 10_000 Hz",
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

fn validate_bandwidth(value: Bandwidth) -> Result<()> {
    if let Bandwidth::ManualHz(hz) = value
        && hz < MIN_BANDWIDTH_HZ
    {
        return Err(Error::invalid_config(
            "bandwidth_hz",
            "manual bandwidth must be at least 1_000 Hz",
        ));
    }
    Ok(())
}

fn validate_gain(gain: GainConfig) -> Result<()> {
    match gain {
        GainConfig::Preset(GainPreset::Linearity(value) | GainPreset::Sensitivity(value))
            if value > MAX_PRESET_GAIN =>
        {
            Err(Error::invalid_config(
                "gain",
                "RFOne preset gain must be in 0..=21",
            ))
        }
        _ => Ok(()),
    }
}
