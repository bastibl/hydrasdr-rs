//! Authoritative active receiver state.

use std::sync::{Arc, Mutex, MutexGuard};

use crate::config::ConfigData;
use crate::{Bandwidth, DecimationMode, Error, GainConfig, Result, RfPort, SampleFormat};

/// Physical RFOne gain stage.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GainStage {
    /// Low-noise amplifier gain.
    Lna,
    /// Mixer gain.
    Mixer,
    /// Variable-gain amplifier gain.
    Vga,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum UnavailableReason {
    NeverApplied,
    UpdateIncomplete,
    LastUpdateFailed,
    PresetNotReported,
}

impl UnavailableReason {
    const fn message(self) -> &'static str {
        match self {
            Self::NeverApplied => "no value has been successfully applied",
            Self::UpdateIncomplete => "an update is in progress or did not complete",
            Self::LastUpdateFailed => "the last update failed",
            Self::PresetNotReported => {
                "physical values are not reported after applying a gain preset"
            }
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ActiveValue<T> {
    Unknown(UnavailableReason),
    Known(T),
}

impl<T: Copy> ActiveValue<T> {
    fn get(self, field: &'static str) -> Result<T> {
        match self {
            Self::Known(value) => Ok(value),
            Self::Unknown(reason) => Err(Error::state_unavailable(field, reason.message())),
        }
    }

    fn get_optional(self, field: &'static str) -> Result<Option<T>> {
        match self {
            Self::Known(value) => Ok(Some(value)),
            Self::Unknown(UnavailableReason::NeverApplied) => Ok(None),
            Self::Unknown(reason) => Err(Error::state_unavailable(field, reason.message())),
        }
    }
}

use UnavailableReason::{
    LastUpdateFailed as LAST_UPDATE_FAILED, NeverApplied as NEVER_APPLIED,
    PresetNotReported as PRESET_NOT_REPORTED, UpdateIncomplete as UPDATE_INCOMPLETE,
};

#[derive(Clone, Debug, Eq, PartialEq)]
struct ActiveStateInner {
    frequency_hz: ActiveValue<u64>,
    sample_rate_hz: ActiveValue<u32>,
    bandwidth: ActiveValue<Bandwidth>,
    sample_format: ActiveValue<SampleFormat>,
    decimation_mode: ActiveValue<DecimationMode>,
    rf_port: ActiveValue<RfPort>,
    lna_gain: ActiveValue<u8>,
    mixer_gain: ActiveValue<u8>,
    vga_gain: ActiveValue<u8>,
    lna_agc: ActiveValue<bool>,
    mixer_agc: ActiveValue<bool>,
    bias_tee: ActiveValue<bool>,
    packing: ActiveValue<bool>,
}

impl Default for ActiveStateInner {
    fn default() -> Self {
        Self {
            frequency_hz: ActiveValue::Unknown(NEVER_APPLIED),
            sample_rate_hz: ActiveValue::Unknown(NEVER_APPLIED),
            bandwidth: ActiveValue::Unknown(NEVER_APPLIED),
            sample_format: ActiveValue::Unknown(NEVER_APPLIED),
            decimation_mode: ActiveValue::Unknown(NEVER_APPLIED),
            rf_port: ActiveValue::Unknown(NEVER_APPLIED),
            lna_gain: ActiveValue::Unknown(NEVER_APPLIED),
            mixer_gain: ActiveValue::Unknown(NEVER_APPLIED),
            vga_gain: ActiveValue::Unknown(NEVER_APPLIED),
            lna_agc: ActiveValue::Unknown(NEVER_APPLIED),
            mixer_agc: ActiveValue::Unknown(NEVER_APPLIED),
            bias_tee: ActiveValue::Unknown(NEVER_APPLIED),
            packing: ActiveValue::Unknown(NEVER_APPLIED),
        }
    }
}

/// Shared view of the active receiver settings.
///
/// HydraSDR firmware does not provide readback requests for these values. This
/// handle therefore records successful operations performed through this crate.
/// A failed update makes the affected value unavailable until a later update
/// succeeds, preventing callers from mistaking stale host state for hardware
/// state. Clones refer to the same authoritative state.
#[derive(Clone, Debug)]
pub struct ActiveState {
    inner: Arc<Mutex<ActiveStateInner>>,
}

impl Default for ActiveState {
    fn default() -> Self {
        Self {
            inner: Arc::new(Mutex::new(ActiveStateInner::default())),
        }
    }
}

impl PartialEq for ActiveState {
    fn eq(&self, other: &Self) -> bool {
        if Arc::ptr_eq(&self.inner, &other.inner) {
            return true;
        }
        let this = self.lock().clone();
        let other = other.lock().clone();
        this == other
    }
}

impl Eq for ActiveState {}

impl ActiveState {
    fn lock(&self) -> MutexGuard<'_, ActiveStateInner> {
        self.inner
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    /// Last successfully applied center frequency in Hz.
    pub fn frequency_hz(&self) -> Result<u64> {
        self.lock().frequency_hz.get("frequency")
    }

    /// Last successfully applied effective sample rate in Hz.
    pub fn sample_rate_hz(&self) -> Result<u32> {
        self.lock().sample_rate_hz.get("sample rate")
    }

    /// Last successfully applied analog-bandwidth policy.
    pub fn bandwidth(&self) -> Result<Bandwidth> {
        self.lock().bandwidth.get("bandwidth")
    }

    /// Last successfully applied high-level sample format.
    pub fn sample_format(&self) -> Result<SampleFormat> {
        self.lock().sample_format.get("sample format")
    }

    /// Last successfully applied host-side decimation policy.
    pub fn decimation_mode(&self) -> Result<DecimationMode> {
        self.lock().decimation_mode.get("decimation mode")
    }

    /// Last successfully selected RF input port.
    pub fn rf_port(&self) -> Result<RfPort> {
        self.lock().rf_port.get("RF port")
    }

    /// Last successfully applied value for one physical gain stage.
    ///
    /// This returns `None` if the stage has never been configured. A failed or
    /// incomplete update returns an error until a later update succeeds.
    pub fn gain(&self, stage: GainStage) -> Result<Option<u8>> {
        let state = self.lock();
        match stage {
            GainStage::Lna => state.lna_gain.get_optional("LNA gain"),
            GainStage::Mixer => state.mixer_gain.get_optional("mixer gain"),
            GainStage::Vga => state.vga_gain.get_optional("VGA gain"),
        }
    }

    /// Last successfully applied common AGC state.
    ///
    /// HydraSDR has separate LNA and mixer AGC controls. This returns an error
    /// if either value is unavailable or the two controls differ.
    pub fn agc_enabled(&self) -> Result<bool> {
        let state = self.lock();
        let lna = state.lna_agc.get("LNA AGC")?;
        let mixer = state.mixer_agc.get("mixer AGC")?;
        if lna == mixer {
            Ok(lna)
        } else {
            Err(Error::state_unavailable(
                "AGC",
                "LNA and mixer AGC states differ",
            ))
        }
    }

    /// Last successfully applied bias-tee state.
    pub fn bias_tee(&self) -> Result<bool> {
        self.lock().bias_tee.get("bias tee")
    }

    /// Last successfully applied packed-transfer state.
    pub fn packing(&self) -> Result<bool> {
        self.lock().packing.get("packing")
    }

    pub(crate) fn apply_config(&self, config: &ConfigData) {
        let mut state = self.lock();
        state.frequency_hz = ActiveValue::Known(config.frequency_hz());
        state.sample_rate_hz = ActiveValue::Known(config.sample_rate_hz());
        state.bandwidth = ActiveValue::Known(config.bandwidth());
        state.sample_format = ActiveValue::Known(config.sample_format());
        state.decimation_mode = ActiveValue::Known(config.decimation_mode());
        if let Some(port) = config.rf_port() {
            state.rf_port = ActiveValue::Known(port);
        }
        apply_gain(&mut state, config.gain());
        if let Some(enabled) = config.bias_tee() {
            state.bias_tee = ActiveValue::Known(enabled);
        }
        state.packing = ActiveValue::Known(config.packing());
    }

    pub(crate) fn begin_config(&self, config: &ConfigData) {
        invalidate_config(&mut self.lock(), config, UPDATE_INCOMPLETE);
    }

    pub(crate) fn fail_config(&self, config: &ConfigData) {
        invalidate_config(&mut self.lock(), config, LAST_UPDATE_FAILED);
    }

    pub(crate) fn begin_frequency_update(&self) {
        self.lock().frequency_hz = ActiveValue::Unknown(UPDATE_INCOMPLETE);
    }

    pub(crate) fn set_frequency_result(&self, value: u64, succeeded: bool) {
        self.lock().frequency_hz = update_result(value, succeeded);
    }

    pub(crate) fn begin_sample_rate_update(&self) {
        self.lock().sample_rate_hz = ActiveValue::Unknown(UPDATE_INCOMPLETE);
    }

    pub(crate) fn set_sample_rate_result(&self, value: u32, succeeded: bool) {
        self.lock().sample_rate_hz = update_result(value, succeeded);
    }

    pub(crate) fn begin_bandwidth_update(&self) {
        self.lock().bandwidth = ActiveValue::Unknown(UPDATE_INCOMPLETE);
    }

    pub(crate) fn set_bandwidth_result(&self, value: u32, succeeded: bool) {
        self.lock().bandwidth = update_result(Bandwidth::ManualHz(value), succeeded);
    }

    pub(crate) fn begin_rf_port_update(&self) {
        self.lock().rf_port = ActiveValue::Unknown(UPDATE_INCOMPLETE);
    }

    pub(crate) fn set_rf_port_result(&self, value: RfPort, succeeded: bool) {
        self.lock().rf_port = update_result(value, succeeded);
    }

    pub(crate) fn begin_gain_update(&self, gain: GainConfig) {
        invalidate_gain(&mut self.lock(), gain, UPDATE_INCOMPLETE);
    }

    pub(crate) fn set_gain_result(&self, gain: GainConfig, succeeded: bool) {
        let mut state = self.lock();
        if succeeded {
            apply_gain(&mut state, gain);
        } else {
            invalidate_gain(&mut state, gain, LAST_UPDATE_FAILED);
        }
    }

    pub(crate) fn begin_bias_tee_update(&self) {
        self.lock().bias_tee = ActiveValue::Unknown(UPDATE_INCOMPLETE);
    }

    pub(crate) fn set_bias_tee_result(&self, enabled: bool, succeeded: bool) {
        self.lock().bias_tee = update_result(enabled, succeeded);
    }
}

fn invalidate_config(state: &mut ActiveStateInner, config: &ConfigData, reason: UnavailableReason) {
    state.frequency_hz = ActiveValue::Unknown(reason);
    state.sample_rate_hz = ActiveValue::Unknown(reason);
    state.bandwidth = ActiveValue::Unknown(reason);
    state.sample_format = ActiveValue::Unknown(reason);
    state.decimation_mode = ActiveValue::Unknown(reason);
    if config.rf_port().is_some() {
        state.rf_port = ActiveValue::Unknown(reason);
    }
    invalidate_gain(state, config.gain(), reason);
    if config.bias_tee().is_some() {
        state.bias_tee = ActiveValue::Unknown(reason);
    }
    state.packing = ActiveValue::Unknown(reason);
}

fn update_result<T>(value: T, succeeded: bool) -> ActiveValue<T> {
    if succeeded {
        ActiveValue::Known(value)
    } else {
        ActiveValue::Unknown(LAST_UPDATE_FAILED)
    }
}

fn apply_gain(state: &mut ActiveStateInner, gain: GainConfig) {
    match gain {
        GainConfig::Unchanged => {}
        GainConfig::Preset(_) => {
            state.lna_gain = ActiveValue::Unknown(PRESET_NOT_REPORTED);
            state.mixer_gain = ActiveValue::Unknown(PRESET_NOT_REPORTED);
            state.vga_gain = ActiveValue::Unknown(PRESET_NOT_REPORTED);
            state.lna_agc = ActiveValue::Unknown(PRESET_NOT_REPORTED);
            state.mixer_agc = ActiveValue::Unknown(PRESET_NOT_REPORTED);
        }
        GainConfig::Manual {
            lna,
            mixer,
            vga,
            lna_agc,
            mixer_agc,
        } => {
            if let Some(value) = lna {
                state.lna_gain = ActiveValue::Known(value);
            }
            if let Some(value) = mixer {
                state.mixer_gain = ActiveValue::Known(value);
            }
            if let Some(value) = vga {
                state.vga_gain = ActiveValue::Known(value);
            }
            if let Some(value) = lna_agc {
                state.lna_agc = ActiveValue::Known(value);
            }
            if let Some(value) = mixer_agc {
                state.mixer_agc = ActiveValue::Known(value);
            }
        }
    }
}

fn invalidate_gain(state: &mut ActiveStateInner, gain: GainConfig, reason: UnavailableReason) {
    match gain {
        GainConfig::Unchanged => {}
        GainConfig::Preset(_) => {
            state.lna_gain = ActiveValue::Unknown(reason);
            state.mixer_gain = ActiveValue::Unknown(reason);
            state.vga_gain = ActiveValue::Unknown(reason);
            state.lna_agc = ActiveValue::Unknown(reason);
            state.mixer_agc = ActiveValue::Unknown(reason);
        }
        GainConfig::Manual {
            lna,
            mixer,
            vga,
            lna_agc,
            mixer_agc,
        } => {
            if lna.is_some() {
                state.lna_gain = ActiveValue::Unknown(reason);
            }
            if mixer.is_some() {
                state.mixer_gain = ActiveValue::Unknown(reason);
            }
            if vga.is_some() {
                state.vga_gain = ActiveValue::Unknown(reason);
            }
            if lna_agc.is_some() {
                state.lna_agc = ActiveValue::Unknown(reason);
            }
            if mixer_agc.is_some() {
                state.mixer_agc = ActiveValue::Unknown(reason);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn failed_update_hides_previous_value_until_success() {
        let state = ActiveState::default();
        state.set_frequency_result(100_000_000, true);
        assert_eq!(state.frequency_hz().unwrap(), 100_000_000);

        state.set_frequency_result(200_000_000, false);
        assert!(state.frequency_hz().is_err());

        state.set_frequency_result(300_000_000, true);
        assert_eq!(state.frequency_hz().unwrap(), 300_000_000);
    }

    #[test]
    fn unapplied_gain_is_optional_but_failed_gain_is_an_error() {
        let state = ActiveState::default();
        assert_eq!(state.gain(GainStage::Lna).unwrap(), None);

        let gain = GainConfig::Manual {
            lna: Some(10),
            mixer: None,
            vga: None,
            lna_agc: None,
            mixer_agc: None,
        };
        state.set_gain_result(gain, false);
        assert!(state.gain(GainStage::Lna).is_err());

        state.set_gain_result(gain, true);
        assert_eq!(state.gain(GainStage::Lna).unwrap(), Some(10));
    }

    #[test]
    fn clones_share_authoritative_state() {
        let state = ActiveState::default();
        let clone = state.clone();

        state.set_sample_rate_result(10_000_000, true);

        assert_eq!(clone.sample_rate_hz().unwrap(), 10_000_000);
    }
}
