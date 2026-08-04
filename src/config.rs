//! Receiver configuration.

use crate::errors::{Error, Result};
use crate::rfone::{
    RFONE_LNA_MAX_GAIN, RFONE_MAX_FREQ_HZ, RFONE_MIN_FREQ_HZ, RFONE_MIXER_MAX_GAIN,
    RFONE_VGA_MAX_GAIN,
};
use crate::types::{DecimationPolicy, SampleType};
use core::marker::PhantomData;

const DEFAULT_FREQUENCY_HZ: u64 = 100_000_000;
const DEFAULT_SAMPLE_RATE_HZ: u32 = 10_000_000;
const MIN_SAMPLE_RATE_HZ: u32 = 10_000;
const MAX_PRESET_GAIN: u8 = 21;
const MAX_VENDOR_INDEX_OR_KHZ: u32 = u16::MAX as u32;
const MAX_RAW_SAMPLE_RATE_HZ: u32 = MAX_VENDOR_INDEX_OR_KHZ * 1_000 + 999;
const MAX_F32_IQ_SAMPLE_RATE_HZ: u32 = (((MAX_VENDOR_INDEX_OR_KHZ + 1) * 1_000) - 1) / 2;

/// Device selection used by [`crate::DeviceBuilder`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum DeviceSelector {
    /// Open the first visible HydraSDR RFOne.
    First,
    /// Open the HydraSDR RFOne with a parsed 64-bit serial number.
    Serial(u64),
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

/// Runtime view of the high-level sample format.
///
/// Configuration uses the [`RawAdc`] and [`F32Iq`] marker types instead. This
/// enum is derived from that marker and is exposed for runtime state reporting.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SampleFormat {
    /// Raw ADC USB blocks measured in real ADC samples per second.
    ///
    /// Use [`crate::Device::sample_rates`] to query the rates advertised by an
    /// opened device.
    RawAdc,
    /// Converted 32-bit float IQ samples measured in complex samples per second.
    ///
    /// The driver may select a higher firmware rate and apply host-side
    /// decimation to produce the requested effective rate. Use
    /// [`crate::Device::sample_rates`] to query the advertised effective rates.
    F32Iq,
}

mod private {
    pub trait Sealed {}
}

/// Compile-time receiver sample mode.
///
/// This trait is sealed because the RFOne high-level API currently supports
/// only [`RawAdc`] and [`F32Iq`].
pub trait SampleMode: private::Sealed + Copy + core::fmt::Debug + Eq {
    /// Runtime format corresponding to this compile-time mode.
    const FORMAT: SampleFormat;
}

/// Raw ADC sample mode.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RawAdc;

/// Converted complex `f32` IQ sample mode.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct F32Iq;

impl private::Sealed for RawAdc {}
impl private::Sealed for F32Iq {}

impl SampleMode for RawAdc {
    const FORMAT: SampleFormat = SampleFormat::RawAdc;
}

impl SampleMode for F32Iq {
    const FORMAT: SampleFormat = SampleFormat::F32Iq;
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
    ///
    /// The preset index must be in the inclusive range `0..=21`.
    Linearity(u8),
    /// Apply the RFOne sensitivity preset table.
    ///
    /// The preset index must be in the inclusive range `0..=21`.
    Sensitivity(u8),
}

/// Complete gain configuration applied after sample/rate/RF-port setup.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GainConfig {
    /// Apply one RFOne preset.
    Preset(GainPreset),
    /// Configure every physical gain stage, including each AGC mode.
    Stages {
        /// LNA gain control.
        lna: StageGain,
        /// Mixer gain control.
        mixer: StageGain,
        /// VGA gain value.
        ///
        /// RFOne accepts values in the inclusive range `0..=15`.
        vga: u8,
    },
}

/// Gain control for an LNA or mixer stage.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StageGain {
    /// Use a fixed stage gain.
    ///
    /// RFOne accepts LNA values in `0..=14` and mixer values in `0..=15`.
    Manual(u8),
    /// Let the hardware AGC control the stage.
    Agc,
}

/// Return the standard fixed-gain configuration.
impl Default for GainConfig {
    fn default() -> Self {
        Self::Stages {
            lna: StageGain::Manual(14),
            mixer: StageGain::Manual(15),
            vga: 6,
        }
    }
}

impl From<GainPreset> for GainConfig {
    fn from(value: GainPreset) -> Self {
        Self::Preset(value)
    }
}

/// Reusable high-level receiver configuration for sample mode `M`.
///
/// The generic mode defaults to [`F32Iq`]. Select [`RawAdc`] with
/// [`ConfigBuilder::raw_adc`].
///
/// The default is complete: it selects RX0, disables the bias tee and both
/// AGCs, and sets 35 dB total gain as LNA 14 dB, mixer 15 dB, and VGA 6 dB.
///
/// Building a config validates static RFOne and USB protocol constraints
/// without opening hardware. It does not prove that connected firmware
/// advertises a sample rate.
/// Every `Config` obtainable through the public API has passed this validation.
///
/// ```
/// use hydrasdr_rs::{Config, GainPreset};
///
/// let config = Config::builder()
///     .raw_adc()
///     .frequency_hz(144_500_000)
///     .sample_rate_hz(10_000_000)
///     .gain(GainPreset::Linearity(10))
///     .build()?;
///
/// assert_eq!(config.frequency_hz(), 144_500_000);
/// # Ok::<(), hydrasdr_rs::Error>(())
/// ```
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Config<M: SampleMode = F32Iq> {
    frequency_hz: u64,
    sample_rate_hz: u32,
    decimation_policy: DecimationPolicy,
    rf_port: Option<RfPort>,
    gain: GainConfig,
    bias_tee: Option<bool>,
    packing: bool,
    mode: PhantomData<fn() -> M>,
}

impl<M: SampleMode> Config<M> {
    fn standard() -> Self {
        Self {
            frequency_hz: DEFAULT_FREQUENCY_HZ,
            sample_rate_hz: DEFAULT_SAMPLE_RATE_HZ,
            decimation_policy: DecimationPolicy::LowBandwidth,
            rf_port: Some(RfPort::Rx0),
            gain: GainConfig::default(),
            bias_tee: Some(false),
            packing: false,
            mode: PhantomData,
        }
    }

    fn into_mode<N: SampleMode>(self) -> Config<N> {
        Config {
            frequency_hz: self.frequency_hz,
            sample_rate_hz: self.sample_rate_hz,
            decimation_policy: self.decimation_policy,
            rf_port: self.rf_port,
            gain: self.gain,
            bias_tee: self.bias_tee,
            packing: self.packing,
            mode: PhantomData,
        }
    }
}

impl Default for Config<F32Iq> {
    fn default() -> Self {
        Self::standard()
    }
}

impl Config<F32Iq> {
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
    pub fn builder() -> ConfigBuilder<F32Iq> {
        ConfigBuilder::default()
    }
}

impl<M: SampleMode> Config<M> {
    /// Tuned center frequency in Hz.
    pub const fn frequency_hz(&self) -> u64 {
        self.frequency_hz
    }

    /// Requested sample rate in Hz.
    ///
    /// For [`SampleFormat::RawAdc`] this is a real ADC rate; for
    /// [`SampleFormat::F32Iq`] it is the effective complex output rate after any
    /// host-side decimation.
    pub const fn sample_rate_hz(&self) -> u32 {
        self.sample_rate_hz
    }

    /// Runtime view of the compile-time sample mode `M`.
    pub const fn sample_format(&self) -> SampleFormat {
        M::FORMAT
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

    fn validate(&self) -> Result<()> {
        validate_frequency(self.frequency_hz)?;
        validate_sample_rate(self.sample_rate_hz, M::FORMAT)?;
        validate_gain(self.gain)?;
        Ok(())
    }

    pub(crate) const fn decimation_policy_internal(&self) -> DecimationPolicy {
        self.decimation_policy
    }

    pub(crate) const fn packing_internal(&self) -> bool {
        self.packing
    }

    pub(crate) fn set_frequency_hz_internal(&mut self, value: u64) {
        self.frequency_hz = value;
    }

    pub(crate) fn set_sample_rate_hz_internal(&mut self, value: u32) {
        self.sample_rate_hz = value;
    }

    pub(crate) fn set_rf_port_internal(&mut self, value: RfPort) {
        self.rf_port = Some(value);
    }

    pub(crate) fn set_gain_internal(&mut self, value: GainConfig) {
        self.gain = value;
    }

    pub(crate) fn apply_internal(&mut self, applied: &Self) {
        self.frequency_hz = applied.frequency_hz;
        self.sample_rate_hz = applied.sample_rate_hz;
        self.decimation_policy = applied.decimation_policy;
        if let Some(port) = applied.rf_port {
            self.rf_port = Some(port);
        }
        self.set_gain_internal(applied.gain);
        if let Some(enabled) = applied.bias_tee {
            self.bias_tee = Some(enabled);
        }
        self.packing = applied.packing;
    }
}

impl Config<RawAdc> {
    /// Whether packed samples should be requested.
    pub const fn packing(&self) -> bool {
        self.packing
    }
}

impl Config<F32Iq> {
    /// Virtual IQ-rate hardware/host decimation policy.
    pub const fn decimation_policy(&self) -> DecimationPolicy {
        self.decimation_policy
    }
}

/// Builder for [`Config`], defaulting to the [`F32Iq`] sample mode.
///
/// ```
/// use hydrasdr_rs::Config;
///
/// let config = Config::builder()
///     .raw_adc()
///     .frequency_hz(915_000_000)
///     .sample_rate_hz(2_000_000)
///     .packing(true)
///     .build()?;
///
/// assert!(config.packing());
/// # Ok::<(), hydrasdr_rs::Error>(())
/// ```
#[derive(Clone, Debug)]
pub struct ConfigBuilder<M: SampleMode = F32Iq> {
    config: Config<M>,
}

impl Default for ConfigBuilder<F32Iq> {
    fn default() -> Self {
        Self {
            config: Config::default(),
        }
    }
}

impl<M: SampleMode> ConfigBuilder<M> {
    /// Set the tuned center frequency in Hz.
    ///
    /// RFOne accepts center frequencies in the inclusive range
    /// `24_000_000..=1_800_000_000` Hz.
    pub fn frequency_hz(mut self, value: u64) -> Self {
        self.config.frequency_hz = value;
        self
    }

    /// Set the requested sample rate in Hz.
    ///
    /// For [`SampleFormat::RawAdc`] this is a real ADC rate; for
    /// [`SampleFormat::F32Iq`] it is the effective complex output rate after any
    /// host-side decimation. [`ConfigBuilder`] validates only that the value is
    /// representable by the USB protocol (`10_000..=65_535_999` Hz for raw ADC
    /// and `10_000..=32_767_999` Hz for F32 IQ). These bounds are not advertised
    /// hardware ranges. Query [`crate::Device::sample_rates`] after opening a
    /// device for its advertised rates; other encodable values are left for
    /// firmware to accept or reject.
    pub fn sample_rate_hz(mut self, value: u32) -> Self {
        self.config.sample_rate_hz = value;
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
    /// Stage gains use the ranges documented on [`StageGain::Manual`].
    pub fn gain(mut self, value: impl Into<GainConfig>) -> Self {
        self.config.gain = value.into();
        self
    }

    /// Enable or disable the RF port bias tee.
    pub fn bias_tee(mut self, enabled: bool) -> Self {
        self.config.bias_tee = Some(enabled);
        self
    }

    /// Validate and build a reusable configuration.
    ///
    /// A successfully built [`Config`] remains valid for its lifetime.
    pub fn build(self) -> Result<Config<M>> {
        self.config.validate()?;
        Ok(self.config)
    }
}

impl ConfigBuilder<RawAdc> {
    /// Enable or disable packed raw-sample transfers.
    pub fn packing(mut self, enabled: bool) -> Self {
        self.config.packing = enabled;
        self
    }
}

impl ConfigBuilder<F32Iq> {
    /// Select raw ADC samples instead of the default converted IQ mode.
    ///
    /// Format-specific options are exposed only after selecting their mode:
    ///
    /// ```compile_fail
    /// use hydrasdr_rs::Config;
    /// let _ = Config::builder().packing(true);
    /// ```
    pub fn raw_adc(mut self) -> ConfigBuilder<RawAdc> {
        self.config.decimation_policy = DecimationPolicy::LowBandwidth;
        ConfigBuilder {
            config: self.config.into_mode(),
        }
    }

    /// Set the firmware/host decimation policy for float IQ samples.
    ///
    /// ```compile_fail
    /// use hydrasdr_rs::{Config, DecimationPolicy};
    /// let _ = Config::builder()
    ///     .raw_adc()
    ///     .decimation_policy(DecimationPolicy::HighDefinition);
    /// ```
    pub fn decimation_policy(mut self, value: DecimationPolicy) -> Self {
        self.config.decimation_policy = value;
        self
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
        GainConfig::Stages {
            lna: StageGain::Manual(value),
            ..
        } if value > RFONE_LNA_MAX_GAIN => Err(Error::invalid_config(
            "gain",
            "manual LNA gain must be in 0..=14",
        )),
        GainConfig::Stages {
            mixer: StageGain::Manual(value),
            ..
        } if value > RFONE_MIXER_MAX_GAIN => Err(Error::invalid_config(
            "gain",
            "manual mixer gain must be in 0..=15",
        )),
        GainConfig::Stages { vga, .. } if vga > RFONE_VGA_MAX_GAIN => Err(Error::invalid_config(
            "gain",
            "manual VGA gain must be in 0..=15",
        )),
        _ => Ok(()),
    }
}
