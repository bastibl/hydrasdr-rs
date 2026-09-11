//! Typed receiver configuration and device discovery/opening.

use crate::{
    config::{
        Config, ConfigBuilder, DeviceSelector, F32Iq, GainConfig, RawAdc, RfPort, SampleFormat,
        SampleMode, validate_frequency, validate_gain,
    },
    errors::Result,
    maybe_future::{MaybeFutureExt, ready},
    protocol::{self, Command, InRequest, OutRequest, ReceiveSettings},
    rfone::{RFONE_F32_IQ_SAMPLE_RATES, RFONE_RAW_ADC_SAMPLE_RATES},
    rx::RxStream,
    session::{RxStreamClaim, Session},
    usb::{ControlBackend, NusbTransport},
};
use nusb::MaybeFuture;
use std::sync::Arc;

/// Owned HydraSDR RFOne device handle.
///
/// The generic mode defaults to [`F32Iq`]. Use [`DeviceBuilder::raw_adc`] to
/// construct a [`Device<RawAdc>`].
///
/// Hardware-opening examples are marked `no_run`; compile-only configuration
/// examples live on [`crate::Config`] and [`crate::ConfigBuilder`].
///
/// ```no_run
/// use hydrasdr_rs::{Device, GainPreset, MaybeFuture, RfPort};
/// use std::time::Duration;
///
/// fn main() -> hydrasdr_rs::Result<()> {
///     let mut dev = Device::builder()
///         .frequency_hz(100_000_000)
///         .sample_rate_hz(10_000_000)
///         .raw_adc()
///         .rf_port(RfPort::Rx0)
///         .gain(GainPreset::Linearity(12))
///         .open()
///         .wait()?;
///
///     let mut rx = dev.rx_stream()?;
///     rx.start().wait()?;
///     if let Some(block) = rx
///         .next_block(Some(Duration::from_secs(1)))
///         .wait()?
///     {
///         println!("{} raw bytes", block.raw_bytes().len());
///     }
///     let stats = rx.close().wait()?;
///     println!("{stats:?}");
///     dev.shutdown().wait()?;
///
///     Ok(())
/// }
/// ```
#[derive(Debug)]
pub struct Device<M: SampleMode = F32Iq> {
    session: Arc<Session>,
    config: Config<M>,
}

impl Device<F32Iq> {
    /// List visible HydraSDR RFOne USB devices without opening them.
    pub fn list() -> impl MaybeFuture<Output = Result<Vec<crate::DeviceDescriptor>>> {
        crate::discovery::list_devices()
    }

    /// Start building and opening a USB device.
    pub fn builder() -> DeviceBuilder<F32Iq> {
        DeviceBuilder::default()
    }

    /// Open the first visible HydraSDR RFOne with default configuration.
    pub fn open() -> impl MaybeFuture<Output = Result<Self>> {
        Self::builder().open()
    }

    /// Open one visible HydraSDR RFOne by serial with default configuration.
    ///
    /// Zero opens any device whose USB serial is absent, invalid, or zero.
    pub fn open_serial(serial: u64) -> impl MaybeFuture<Output = Result<Self>> {
        Self::builder().serial(serial).open()
    }

    /// Ask the browser to grant WebUSB access to a HydraSDR without opening it.
    ///
    /// Call this from a browser-window user gesture. After permission is granted,
    /// [`Device::open`] may discover and open the device from a Web Worker.
    #[cfg(target_arch = "wasm32")]
    pub async fn request_permission() -> Result<()> {
        Self::builder().request_permission().await
    }
}

impl<M: SampleMode> Device<M> {
    /// Return immutable metadata read while opening the device.
    pub fn info(&self) -> &DeviceInfo {
        self.session.info.get().expect("opened device has metadata")
    }

    /// Return the configuration last successfully applied through this driver.
    ///
    /// RFOne's write-only controls do not provide hardware readback. A failed or
    /// cancelled configuration operation leaves this snapshot unchanged.
    pub fn config(&self) -> &Config<M> {
        &self.config
    }

    /// Return the fixed sample rates supported for the active sample format.
    ///
    /// Converted F32 IQ results include exactly representable effective rates
    /// produced by host-side decimation. This accessor performs no USB requests.
    pub fn sample_rates(&self) -> Vec<u32> {
        match M::FORMAT {
            SampleFormat::RawAdc => RFONE_RAW_ADC_SAMPLE_RATES.to_vec(),
            SampleFormat::F32Iq => RFONE_F32_IQ_SAMPLE_RATES.to_vec(),
        }
    }

    /// Apply a receiver configuration of the device's sample mode.
    ///
    /// A configuration for another mode is rejected at compile time:
    ///
    /// ```compile_fail
    /// use hydrasdr_rs::{Config, Device, MaybeFuture, RawAdc};
    /// fn configure_f32_as_raw(device: &mut Device<RawAdc>) {
    ///     let f32_config = Config::builder().build().unwrap();
    ///     device.configure(&f32_config).wait().unwrap();
    /// }
    /// ```
    pub fn configure<'a>(
        &'a mut self,
        config: &'a Config<M>,
    ) -> impl MaybeFuture<Output = Result<()>> + 'a {
        let (commands, receive) = protocol::configuration(config);
        self.update(config.clone(), commands, Some(receive), Ok(()))
    }

    /// Set only the tuned center frequency.
    pub fn set_frequency_hz(&mut self, frequency_hz: u64) -> impl MaybeFuture<Output = Result<()>> {
        let mut config = self.config.clone();
        config.set_frequency_hz_internal(frequency_hz);
        self.update(
            config,
            vec![OutRequest::set_frequency(frequency_hz).into()],
            None,
            validate_frequency(frequency_hz),
        )
    }

    /// Set only the requested sample rate.
    ///
    /// The value is a real ADC rate for [`SampleFormat::RawAdc`] and an
    /// effective complex output rate for [`SampleFormat::F32Iq`]. The value must
    /// be one of the fixed rates returned by [`Device::sample_rates`].
    pub fn set_sample_rate_hz(
        &mut self,
        sample_rate_hz: u32,
    ) -> impl MaybeFuture<Output = Result<()>> {
        let mut config = self.config.clone();
        config.set_sample_rate_hz_internal(sample_rate_hz);
        self.update_rate(config)
    }

    /// Set only the selected RF input port.
    pub fn set_rf_port(&mut self, port: RfPort) -> impl MaybeFuture<Output = Result<()>> {
        let mut config = self.config.clone();
        config.set_rf_port_internal(port);
        self.update(
            config,
            vec![InRequest::set_rf_port(port).into()],
            None,
            Ok(()),
        )
    }

    /// Replace the complete gain configuration.
    pub fn set_gain(&mut self, gain: GainConfig) -> impl MaybeFuture<Output = Result<()>> {
        let mut config = self.config.clone();
        config.set_gain_internal(gain);
        let commands = protocol::gain_config_plan(gain)
            .into_iter()
            .map(Command::from)
            .collect();
        self.update(config, commands, None, validate_gain(gain))
    }

    /// Enable or disable RF input bias power.
    pub fn set_bias_tee(&mut self, enabled: bool) -> impl MaybeFuture<Output = Result<()>> {
        let mut config = self.config.clone();
        config.set_bias_tee_internal(enabled);
        self.update(
            config,
            vec![OutRequest::set_rf_bias(enabled).into()],
            None,
            Ok(()),
        )
    }

    /// Turn off reception and RF input bias power.
    ///
    /// Returns [`crate::Error::Busy`] without sending shutdown commands while the
    /// stream claim is held, including by dormant or stopped streams, close
    /// operations, and background cleanup.
    /// Call [`RxStream::close`] to release it. A Busy rejection leaves the device
    /// usable. An unpolled operation has no lifecycle effects; once execution
    /// begins without a claim, shutdown is terminal and retryable.
    ///
    /// Both shutdown commands are attempted even if turning off the receiver
    /// fails. Await this operation in async code, or call [`MaybeFuture::wait`]
    /// on native targets. Native drops perform the same sequence best-effort;
    /// WebUSB drops schedule it as a background operation. Streams keep the
    /// device alive, so dropping this handle does not interrupt reception or
    /// prevent a stream from restarting. Final cleanup happens when the last
    /// device or stream owner is dropped. Explicit shutdown is required when
    /// the caller must observe completion or an error. A failed or canceled
    /// shutdown can be retried and leaves the final owner's drop fallback armed.
    #[must_use = "shutdown must be awaited or waited to send hardware cleanup commands"]
    pub fn shutdown(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        self.session.shutdown()
    }

    /// Create the device's exclusively claimed typed receive stream.
    ///
    /// The device remains available for live frequency, sample-rate, RF-port,
    /// and gain changes while reception is active. The stream starts lazily
    /// when [`RxStream::start`] is waited or awaited. Only one stream may be
    /// claimed at a time. A stream keeps the device alive even after this handle
    /// is dropped, including while the stream is stopped. Dropping the stream
    /// releases the claim after best-effort receiver cleanup; WebUSB performs
    /// that cleanup in the background.
    pub fn rx_stream(&self) -> Result<RxStream<M>> {
        Ok(RxStream::new(RxStreamClaim::acquire(&self.session)?))
    }
}

impl Device<RawAdc> {
    /// Enable or disable packed raw-sample transfers.
    ///
    /// An active raw stream adopts the new transfer format on its next block.
    pub fn set_packing(&mut self, enabled: bool) -> impl MaybeFuture<Output = Result<()>> {
        let mut config = self.config.clone();
        config.set_packing_internal(enabled);
        let receive = ReceiveSettings {
            packing: enabled,
            ..self.session.receive_settings()
        };
        self.update(
            config,
            vec![InRequest::set_packing(enabled).into()],
            Some(receive),
            Ok(()),
        )
    }
}

impl Device<F32Iq> {
    /// Change the firmware/host decimation policy and reapply the requested IQ rate.
    ///
    /// An active converted stream adopts the resulting host decimation factor
    /// on its next read.
    pub fn set_decimation_policy(
        &mut self,
        policy: crate::DecimationPolicy,
    ) -> impl MaybeFuture<Output = Result<()>> {
        let mut config = self.config.clone();
        config.set_decimation_policy_internal(policy);
        self.update_rate(config)
    }
}

impl<M: SampleMode> Device<M> {
    fn update(
        &mut self,
        proposed: Config<M>,
        commands: Vec<Command>,
        receive: Option<ReceiveSettings>,
        validation: Result<()>,
    ) -> impl MaybeFuture<Output = Result<()>> + '_ {
        apply_update(
            &self.session,
            &mut self.config,
            proposed,
            commands,
            receive,
            validation,
        )
    }

    fn update_rate(&mut self, proposed: Config<M>) -> impl MaybeFuture<Output = Result<()>> + '_ {
        let rate = protocol::sample_rate_config(
            M::FORMAT,
            proposed.decimation_policy_internal(),
            proposed.sample_rate_hz(),
        );
        ready(rate).and_then(move |rate| {
            let receive = ReceiveSettings {
                decimation: rate.decimation as usize,
                ..self.session.receive_settings()
            };
            self.update(
                proposed,
                vec![InRequest::set_samplerate(rate.param).into()],
                Some(receive),
                Ok(()),
            )
        })
    }
}

/// Builder that selects, opens, and initially configures a `nusb` device.
///
/// The generic mode defaults to [`F32Iq`].
///
/// Use [`DeviceBuilder::config`] to validate the same settings without
/// opening hardware, or [`DeviceBuilder::open`] to apply them to a selected RFOne.
///
/// ```
/// use hydrasdr_rs::{Device, GainPreset};
///
/// let config = Device::builder()
///     .raw_adc()
///     .frequency_hz(433_920_000)
///     .sample_rate_hz(5_000_000)
///     .gain(GainPreset::Sensitivity(8))
///     .config()?;
///
/// assert_eq!(config.frequency_hz(), 433_920_000);
/// # Ok::<(), hydrasdr_rs::Error>(())
/// ```
#[derive(Clone, Debug)]
pub struct DeviceBuilder<M: SampleMode = F32Iq> {
    selector: DeviceSelector,
    config: ConfigBuilder<M>,
}

impl Default for DeviceBuilder<F32Iq> {
    fn default() -> Self {
        Self {
            selector: DeviceSelector::First,
            config: ConfigBuilder::default(),
        }
    }
}

impl<M: SampleMode> DeviceBuilder<M> {
    /// Select a device by normalized 64-bit serial number.
    ///
    /// Zero selects any device whose USB serial is absent, invalid, or zero.
    pub fn serial(mut self, serial: u64) -> Self {
        self.selector = DeviceSelector::Serial(serial);
        self
    }

    /// Ask the browser to grant WebUSB access to the selected HydraSDR.
    ///
    /// This only performs the browser permission request; it does not open or
    /// configure the device. Call it from a browser-window user gesture, then
    /// use [`DeviceBuilder::open`] from either the window or a Web Worker.
    #[cfg(target_arch = "wasm32")]
    pub async fn request_permission(&self) -> Result<()> {
        let serial = match self.selector {
            DeviceSelector::First => None,
            DeviceSelector::Serial(serial) => Some(serial),
        };
        crate::discovery::request_device_permission(serial).await
    }

    /// Set the tuned center frequency in Hz.
    ///
    /// RFOne accepts center frequencies in the inclusive range
    /// `24_000_000..=1_800_000_000` Hz.
    pub fn frequency_hz(mut self, value: u64) -> Self {
        self.config = self.config.frequency_hz(value);
        self
    }

    /// Set the requested sample rate in Hz.
    ///
    /// For [`SampleFormat::RawAdc`] this is a real ADC rate; for
    /// [`SampleFormat::F32Iq`] it is the effective complex output rate after any
    /// host-side decimation. The builder accepts exactly the fixed rates
    /// returned by [`Device::sample_rates`] for the selected format.
    pub fn sample_rate_hz(mut self, value: u32) -> Self {
        self.config = self.config.sample_rate_hz(value);
        self
    }

    /// Select the RF input port.
    pub fn rf_port(mut self, value: crate::RfPort) -> Self {
        self.config = self.config.rf_port(value);
        self
    }

    /// Set the gain configuration.
    ///
    /// Preset gain indexes must be in the inclusive range `0..=21`.
    /// Stage gains use the ranges documented on [`crate::StageGain::Manual`].
    pub fn gain(mut self, value: impl Into<crate::GainConfig>) -> Self {
        self.config = self.config.gain(value);
        self
    }

    /// Enable or disable the RF port bias tee.
    pub fn bias_tee(mut self, enabled: bool) -> Self {
        self.config = self.config.bias_tee(enabled);
        self
    }

    /// Build the reusable configuration represented by this builder.
    pub fn config(self) -> Result<Config<M>> {
        self.config.build()
    }

    /// Open and configure the selected device.
    ///
    /// Await this operation in async code, or call [`MaybeFuture::wait`] on native targets.
    /// After USB open succeeds, configuration failure or cancellation performs
    /// receiver-off and bias-off cleanup. WebUSB cancellation schedules that
    /// cleanup in the background.
    pub fn open(self) -> impl MaybeFuture<Output = Result<Device<M>>> {
        let serial = match self.selector {
            DeviceSelector::First => None,
            DeviceSelector::Serial(serial) => Some(serial),
        };
        ready(self.config.build()).and_then(move |config| {
            NusbTransport::open(serial).and_then(move |control| {
                Session::initialize(control, config.clone())
                    .map_ok(move |session| Device { session, config })
            })
        })
    }
}

impl DeviceBuilder<RawAdc> {
    /// Enable or disable packed raw-sample transfers.
    pub fn packing(mut self, enabled: bool) -> Self {
        self.config = self.config.packing(enabled);
        self
    }
}

impl DeviceBuilder<F32Iq> {
    /// Select raw ADC samples instead of the default converted IQ mode.
    pub fn raw_adc(self) -> DeviceBuilder<RawAdc> {
        DeviceBuilder {
            selector: self.selector,
            config: self.config.raw_adc(),
        }
    }

    /// Set the firmware/host decimation policy for float IQ samples.
    pub fn decimation_policy(mut self, value: crate::DecimationPolicy) -> Self {
        self.config = self.config.decimation_policy(value);
        self
    }
}

/// RF port metadata returned in [`DeviceInfo`].
#[derive(Clone, Debug, PartialEq)]
pub struct RfPortInfo {
    /// Protocol selector used to activate this port.
    pub port: crate::RfPort,
    /// Firmware-provided RF port name.
    pub name: &'static str,
    /// Minimum tunable frequency for this port, in Hz.
    pub min_frequency: u64,
    /// Maximum tunable frequency for this port, in Hz.
    pub max_frequency: u64,
    /// Whether this port exposes a bias tee.
    pub has_bias_tee: bool,
}

/// Immutable device metadata reported while opening the device.
#[derive(Clone, Debug, PartialEq)]
pub struct DeviceInfo {
    /// Board name detected from the firmware board ID.
    pub board_name: &'static str,
    /// Firmware version string.
    pub firmware_version: String,
    /// Parsed 64-bit serial number, if available.
    pub serial: Option<u64>,
    /// RF ports reported for this device.
    pub rf_ports: Vec<RfPortInfo>,
}

/// Keep the public snapshot and receive settings committed by the same completed operation.
fn apply_update<'a, C: ControlBackend + 'static, M: SampleMode>(
    session: &Arc<Session<C>>,
    active: &'a mut Config<M>,
    proposed: Config<M>,
    commands: Vec<Command>,
    receive: Option<ReceiveSettings>,
    validation: Result<()>,
) -> impl MaybeFuture<Output = Result<()>> + use<'a, C, M> {
    let operation = session.apply(commands, receive);
    ready(validation)
        .and_then(move |_| operation)
        .map(move |result| {
            result?;
            active.apply_internal(&proposed);
            Ok(())
        })
}

#[cfg(all(test, not(target_arch = "wasm32")))]
mod tests {
    use super::*;
    use crate::{protocol::ReceiverMode, test_support::FakeTransport};
    use futures_lite::future::{block_on, poll_once};
    use std::{future::IntoFuture, sync::atomic::Ordering};

    #[test]
    fn configuration_commits_snapshot_and_receive_settings_only_on_success() {
        block_on(async {
            for wait in [false, true] {
                let session = Session::new(FakeTransport::default());
                let state = &session.control.state;
                let mut active = Config::default();
                let initial = active.clone();
                let proposed = Config::builder()
                    .frequency_hz(144_000_000)
                    .sample_rate_hz(625_000)
                    .build()
                    .unwrap();
                state.fail_packing.store(true, Ordering::SeqCst);
                let (commands, receive) = protocol::configuration(&proposed);
                let operation = apply_update(
                    &session,
                    &mut active,
                    proposed.clone(),
                    commands,
                    Some(receive),
                    Ok(()),
                );
                assert!(
                    if wait {
                        operation.wait()
                    } else {
                        operation.await
                    }
                    .is_err()
                );
                assert_eq!(active, initial);
                assert_eq!(session.receive_settings().decimation, 1);
                state.fail_packing.store(false, Ordering::SeqCst);
                let (commands, receive) = protocol::configuration(&proposed);
                let operation = apply_update(
                    &session,
                    &mut active,
                    proposed.clone(),
                    commands,
                    Some(receive),
                    Ok(()),
                );
                if wait {
                    operation.wait().unwrap();
                } else {
                    operation.await.unwrap();
                }
                assert_eq!(active, proposed);
                assert_eq!(session.receive_settings().decimation, 4);
            }
        });
    }

    #[test]
    fn canceled_and_unpolled_configuration_leave_snapshot_and_receive_settings_unchanged() {
        block_on(async {
            for polled in [false, true] {
                let session = Session::new(FakeTransport::default());
                let state = &session.control.state;
                let mut active = Config::default();
                let proposed = Config::builder().sample_rate_hz(625_000).build().unwrap();
                state.pause_control_in_at.store(2, Ordering::SeqCst);
                let (commands, receive) = protocol::configuration(&proposed);
                let mut operation = Box::pin(
                    apply_update(
                        &session,
                        &mut active,
                        proposed,
                        commands,
                        Some(receive),
                        Ok(()),
                    )
                    .into_future(),
                );
                if polled {
                    assert!(poll_once(&mut operation).await.is_none());
                }
                drop(operation);
                assert_eq!(active, Config::default());
                assert_eq!(session.receive_settings().decimation, 1);
                if !polled {
                    assert!(state.control_in_requests.lock().unwrap().is_empty());
                    assert!(state.control_out_requests.lock().unwrap().is_empty());
                }
            }
        });
    }

    #[test]
    fn invalid_configuration_change_sends_no_commands() {
        let session = Session::new(FakeTransport::default());
        let mut config = Config::default();
        let proposed = config.clone();
        assert!(
            apply_update(
                &session,
                &mut config,
                proposed,
                vec![OutRequest::set_frequency(0).into()],
                None,
                validate_frequency(0)
            )
            .wait()
            .is_err()
        );
        assert!(
            session
                .control
                .state
                .control_out_requests
                .lock()
                .unwrap()
                .is_empty()
        );
        assert_eq!(config, Config::default());
    }

    #[test]
    fn opening_reads_firmware_once_and_uses_its_metadata() {
        let control = FakeTransport::default();
        let state = Arc::clone(&control.state);
        let session = Session::initialize(control, Config::default())
            .wait()
            .unwrap();
        assert_eq!(
            session.info.get().unwrap().firmware_version,
            "HydraSDR RF test"
        );
        assert_eq!(
            state
                .control_in_requests
                .lock()
                .unwrap()
                .iter()
                .filter(|r| r.request == protocol::VendorRequest::VersionStringRead)
                .count(),
            1
        );
        session.shutdown().wait().unwrap();
    }

    #[test]
    fn metadata_failure_or_cancellation_shuts_down_claimed_hardware() {
        block_on(async {
            for cancel in [false, true] {
                for stage in 1..=3 {
                    let control = FakeTransport::default();
                    let state = Arc::clone(&control.state);
                    if cancel {
                        state.pause_control_in_at.store(stage, Ordering::SeqCst);
                    } else {
                        let mut replies = state.responses.lock().unwrap();
                        if stage > 1 {
                            replies.push_back(Ok(vec![1]));
                        }
                        if stage > 2 {
                            replies.push_back(Ok(b"HydraSDR RF test".to_vec()));
                        }
                        replies.push_back(Err(nusb::transfer::TransferError::Disconnected.into()));
                    }
                    let mut operation =
                        Box::pin(Session::initialize(control, Config::default()).into_future());
                    if cancel {
                        assert!(poll_once(&mut operation).await.is_none());
                        drop(operation);
                    } else {
                        assert!(operation.await.is_err());
                    }
                    assert_eq!(
                        *state.control_out_requests.lock().unwrap(),
                        [
                            OutRequest::receiver_mode(ReceiverMode::Off),
                            OutRequest::set_rf_bias(false)
                        ]
                    );
                }
            }
        });
    }
}
