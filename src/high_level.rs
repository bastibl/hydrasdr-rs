//! High-level synchronous and asynchronous HydraSDR RFOne driver interface.

use core::marker::PhantomData;
use nusb::MaybeFuture;
use std::future::{Future, IntoFuture};
use std::pin::Pin;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

use crate::Complex32;
use crate::commands::ReceiverMode;
use crate::config::{
    Config, ConfigBuilder, DeviceSelector, F32Iq, GainConfig, RawAdc, RfPort, SampleFormat,
    SampleMode, validate_frequency, validate_gain, validate_sample_rate,
};
use crate::device::HydraSdr;
use crate::errors::{Error, Result};
use crate::maybe_future::{Either, MaybeFutureExt, ready};
use std::time::Duration;

use crate::streaming::{
    AsyncDirectRxStream, AsyncRawRxStream as DirectAsyncRawRxStream, AsyncStreamingBackend,
    StreamingStats, Transfer,
};
#[cfg(not(target_arch = "wasm32"))]
use crate::streaming::{DirectRxStream, RawRxStream as DirectRawRxStream, StreamingBackend};
use crate::types::DeviceInfo;
use crate::usb::control::{ControlBackend, NusbControl};

/// High-level owned HydraSDR RFOne device handle.
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
///     let stats = rx.stop().wait()?;
///     println!("{stats:?}");
///     drop(rx);
///     dev.shutdown().wait()?;
///
///     Ok(())
/// }
/// ```
#[derive(Debug)]
pub(crate) struct DeviceInner<C: ControlBackend = NusbControl> {
    direct: HydraSdr<C>,
    info: DeviceInfo,
    #[cfg(not(target_arch = "wasm32"))]
    shutdown_on_drop: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum DeviceLifecycle {
    Open,
    Closing,
    /// The device handle was dropped while a receiver lease was held.
    /// That lease now owns the final receiver-off and bias-off cleanup.
    DropCleanupPending,
    Closed,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ReceiverSlot {
    /// No operation can have left the hardware receiver enabled.
    Idle,
    /// One stream owns the right and obligation to turn the receiver off.
    Held,
    /// The owning stream was dropped without confirming receiver-off cleanup.
    Orphaned,
}

#[derive(Debug)]
struct SharedDeviceState {
    device: DeviceLifecycle,
    /// This tracks cleanup ownership, not USB queue or read state.
    receiver: ReceiverSlot,
    stream_claimed: bool,
}

impl Default for SharedDeviceState {
    fn default() -> Self {
        Self {
            device: DeviceLifecycle::Open,
            receiver: ReceiverSlot::Idle,
            stream_claimed: false,
        }
    }
}

type SharedState = Arc<Mutex<SharedDeviceState>>;

fn lock_shared(state: &SharedState) -> MutexGuard<'_, SharedDeviceState> {
    state.lock().unwrap_or_else(PoisonError::into_inner)
}

fn ensure_device_open(state: &SharedState) -> Result<()> {
    if lock_shared(state).device == DeviceLifecycle::Open {
        Ok(())
    } else {
        Err(Error::DeviceClosed)
    }
}

fn begin_shutdown(state: &SharedState) -> Result<bool> {
    let mut shared = lock_shared(state);
    match shared.device {
        DeviceLifecycle::DropCleanupPending | DeviceLifecycle::Closed => Ok(false),
        DeviceLifecycle::Closing => Ok(true),
        DeviceLifecycle::Open => {
            if shared.receiver == ReceiverSlot::Held {
                return Err(Error::Busy);
            }
            shared.device = DeviceLifecycle::Closing;
            Ok(true)
        }
    }
}

fn finish_shutdown(state: &SharedState) {
    let mut shared = lock_shared(state);
    shared.device = DeviceLifecycle::Closed;
    shared.receiver = ReceiverSlot::Idle;
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum DeviceDropAction {
    None,
    Immediate,
    DeferredToReceiver,
}

fn begin_device_drop(state: &SharedState) -> DeviceDropAction {
    let mut shared = lock_shared(state);
    match shared.device {
        DeviceLifecycle::Closed => DeviceDropAction::None,
        DeviceLifecycle::DropCleanupPending => DeviceDropAction::DeferredToReceiver,
        DeviceLifecycle::Open | DeviceLifecycle::Closing => {
            if shared.receiver == ReceiverSlot::Held {
                shared.device = DeviceLifecycle::DropCleanupPending;
                DeviceDropAction::DeferredToReceiver
            } else {
                shared.device = DeviceLifecycle::Closed;
                shared.receiver = ReceiverSlot::Idle;
                DeviceDropAction::Immediate
            }
        }
    }
}

fn finish_deferred_shutdown(state: &SharedState) {
    let mut shared = lock_shared(state);
    if shared.device == DeviceLifecycle::DropCleanupPending {
        shared.device = DeviceLifecycle::Closed;
        shared.receiver = ReceiverSlot::Idle;
    }
}

fn complete_shutdown(state: &SharedState, result: Result<()>) -> Result<()> {
    if result.is_ok() {
        finish_shutdown(state);
    }
    result
}

/// High-level owned HydraSDR RFOne device handle.
#[derive(Debug)]
pub struct Device<M: SampleMode = F32Iq> {
    inner: DeviceInner<NusbControl>,
    config: Config<M>,
    shared: SharedState,
}

impl Device<F32Iq> {
    /// List visible HydraSDR RFOne USB devices without opening them.
    pub fn list() -> impl MaybeFuture<Output = Result<Vec<crate::DeviceDescriptor>>> {
        crate::discovery::list_devices()
    }

    /// Start building and opening a high-level USB device.
    pub fn builder() -> DeviceBuilder<F32Iq> {
        DeviceBuilder::default()
    }

    /// Open the first visible HydraSDR RFOne with default high-level configuration.
    pub fn open() -> impl MaybeFuture<Output = Result<Self>> {
        Self::builder().open()
    }

    /// Open one visible HydraSDR RFOne by serial with default high-level configuration.
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
        self.inner.info()
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
        self.inner.sample_rates()
    }

    /// Apply a high-level receiver configuration of the device's sample mode.
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
        let lifecycle = ensure_device_open(&self.shared);
        let applied = config.clone();
        let active = &mut self.config;
        let operation = self.inner.configure(config);
        ready(lifecycle)
            .and_then(move |()| operation)
            .map(move |result| {
                if result.is_ok() {
                    active.apply_internal(&applied);
                }
                result
            })
    }

    /// Set only the tuned center frequency.
    pub fn set_frequency_hz(&mut self, frequency_hz: u64) -> impl MaybeFuture<Output = Result<()>> {
        let lifecycle = ensure_device_open(&self.shared);
        let config = &mut self.config;
        let operation = self.inner.set_frequency_hz(frequency_hz);
        ready(lifecycle)
            .and_then(move |()| operation)
            .map(move |result| {
                if result.is_ok() {
                    config.set_frequency_hz_internal(frequency_hz);
                }
                result
            })
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
        let lifecycle = ensure_device_open(&self.shared);
        let config = &mut self.config;
        let operation = self.inner.set_sample_rate_hz(sample_rate_hz, M::FORMAT);
        ready(lifecycle)
            .and_then(move |()| operation)
            .map(move |result| {
                if result.is_ok() {
                    config.set_sample_rate_hz_internal(sample_rate_hz);
                }
                result
            })
    }

    /// Set only the selected RF input port.
    pub fn set_rf_port(&mut self, port: RfPort) -> impl MaybeFuture<Output = Result<()>> {
        let lifecycle = ensure_device_open(&self.shared);
        let config = &mut self.config;
        let operation = self.inner.set_rf_port(port);
        ready(lifecycle)
            .and_then(move |()| operation)
            .map(move |result| {
                if result.is_ok() {
                    config.set_rf_port_internal(port);
                }
                result
            })
    }

    /// Replace the complete gain configuration.
    pub fn set_gain(&mut self, gain: GainConfig) -> impl MaybeFuture<Output = Result<()>> {
        let lifecycle = ensure_device_open(&self.shared);
        let config = &mut self.config;
        let operation = self.inner.set_gain(gain);
        ready(lifecycle)
            .and_then(move |()| operation)
            .map(move |result| {
                if result.is_ok() {
                    config.set_gain_internal(gain);
                }
                result
            })
    }

    /// Enable or disable RF input bias power.
    pub fn set_bias_tee(&mut self, enabled: bool) -> impl MaybeFuture<Output = Result<()>> {
        let lifecycle = ensure_device_open(&self.shared);
        let config = &mut self.config;
        let operation = self.inner.set_bias_tee(enabled);
        ready(lifecycle)
            .and_then(move |()| operation)
            .map(move |result| {
                if result.is_ok() {
                    config.set_bias_tee_internal(enabled);
                }
                result
            })
    }

    /// Turn off reception and RF input bias power.
    ///
    /// Returns [`Error::Busy`] without sending shutdown commands while the
    /// receive stream is active or still requires cleanup. Stop the stream and
    /// retry. A stopped stream may remain claimed, but cannot be restarted once
    /// shutdown begins.
    ///
    /// Both shutdown commands are attempted even if turning off the receiver
    /// fails. Await this operation in async code, or call [`MaybeFuture::wait`]
    /// on native targets. Native drops perform the same sequence best-effort;
    /// WebUSB drops schedule it as a background operation. If a stream is
    /// starting or active, dropping the device transfers final cleanup to that
    /// stream so receiver-off cannot race a pending receiver-on command. Explicit
    /// shutdown is still required when the caller must observe completion or an
    /// error. On native targets, dropping or canceling the returned operation
    /// keeps the blocking fallback armed. A canceled WebUSB shutdown may be
    /// retried; dropping the device schedules a background cleanup attempt.
    #[must_use = "shutdown must be awaited or waited to send hardware cleanup commands"]
    pub fn shutdown(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        let decision = begin_shutdown(&self.shared);
        let shared = Arc::clone(&self.shared);
        let operation = self.inner.shutdown();
        ready(decision).and_then(move |run| {
            if run {
                Either::left(operation.map(move |result| complete_shutdown(&shared, result)))
            } else {
                Either::right(ready(Ok(())))
            }
        })
    }

    /// Create the device's exclusively claimed typed receive stream.
    ///
    /// The device remains available for live frequency, sample-rate, RF-port,
    /// and gain changes while reception is active. The stream starts lazily
    /// when [`RxStream::start`] is waited or awaited. Only one stream may be
    /// claimed at a time; dropping it releases the claim.
    pub fn rx_stream(&self) -> Result<RxStream<M>> {
        let claim = RxStreamClaim::acquire(&self.shared)?;
        Ok(RxStream::new(
            self.inner.stream_handle(),
            Arc::clone(&self.shared),
            claim,
        ))
    }
}

impl Device<RawAdc> {
    /// Enable or disable packed raw-sample transfers.
    ///
    /// An active raw stream adopts the new transfer format on its next block.
    pub fn set_packing(&mut self, enabled: bool) -> impl MaybeFuture<Output = Result<()>> {
        let lifecycle = ensure_device_open(&self.shared);
        let config = &mut self.config;
        let operation = self.inner.set_packing(enabled);
        ready(lifecycle)
            .and_then(move |()| operation)
            .map(move |result| {
                if result.is_ok() {
                    config.set_packing_internal(enabled);
                }
                result
            })
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
        let lifecycle = ensure_device_open(&self.shared);
        let config = &mut self.config;
        let sample_rate_hz = config.sample_rate_hz();
        let operation = self.inner.set_decimation_policy(sample_rate_hz, policy);
        ready(lifecycle)
            .and_then(move |()| operation)
            .map(move |result| {
                if result.is_ok() {
                    config.set_decimation_policy_internal(policy);
                }
                result
            })
    }
}

impl<M: SampleMode> Drop for Device<M> {
    fn drop(&mut self) {
        let action = begin_device_drop(&self.shared);
        #[cfg(not(target_arch = "wasm32"))]
        if action != DeviceDropAction::Immediate {
            self.inner.shutdown_on_drop = false;
        }
        #[cfg(target_arch = "wasm32")]
        if action == DeviceDropAction::Immediate {
            let direct = self.inner.direct.stream_handle();
            wasm_bindgen_futures::spawn_local(async move {
                let _ = shutdown_hardware(&direct).await;
            });
        }
    }
}

fn shutdown_hardware<C: ControlBackend>(
    direct: &HydraSdr<C>,
) -> impl MaybeFuture<Output = Result<()>> + use<C> {
    let receiver_off = direct
        .receiver_mode(ReceiverMode::Off)
        .map_err(|error| error.at("turning off the receiver during shutdown"));
    let bias_off = direct
        .set_rf_bias(false)
        .map_err(|error| error.at("turning off RF bias power during shutdown"));

    receiver_off
        .map(Ok::<_, Error>)
        .and_then(move |receiver_result| {
            bias_off.map(move |bias_result| receiver_result.and(bias_result))
        })
}

struct OpenCleanupGuard<C: ControlBackend + 'static> {
    direct: HydraSdr<C>,
    armed: bool,
}

impl<C: ControlBackend + 'static> OpenCleanupGuard<C> {
    fn new(direct: &HydraSdr<C>) -> Self {
        Self {
            direct: direct.stream_handle(),
            armed: true,
        }
    }

    fn disarm(mut self) {
        self.armed = false;
    }

    fn cleanup(self) -> impl MaybeFuture<Output = Result<()>> + use<C> {
        let operation = shutdown_hardware(&self.direct);
        operation.map(move |result| match result {
            Ok(()) => {
                self.disarm();
                Ok(())
            }
            Err(error) => Err(error),
        })
    }
}

impl<C: ControlBackend + 'static> Drop for OpenCleanupGuard<C> {
    fn drop(&mut self) {
        #[cfg(not(target_arch = "wasm32"))]
        if self.armed {
            self.armed = false;
            let _ = shutdown_hardware(&self.direct).wait();
        }
        #[cfg(target_arch = "wasm32")]
        if self.armed {
            self.armed = false;
            let direct = self.direct.stream_handle();
            wasm_bindgen_futures::spawn_local(async move {
                let _ = shutdown_hardware(&direct).await;
            });
        }
    }
}

struct DeferredShutdownGuard<C: ControlBackend + 'static> {
    direct: HydraSdr<C>,
    shared: SharedState,
    armed: bool,
}

impl<C: ControlBackend + 'static> DeferredShutdownGuard<C> {
    fn new(direct: HydraSdr<C>, shared: &SharedState) -> Self {
        Self {
            direct,
            shared: Arc::clone(shared),
            armed: true,
        }
    }

    fn complete(mut self) {
        finish_deferred_shutdown(&self.shared);
        self.armed = false;
    }

    fn cleanup(self) -> impl MaybeFuture<Output = Result<()>> + use<C> {
        let operation = shutdown_hardware(&self.direct);
        operation.map(move |result| match result {
            Ok(()) => {
                self.complete();
                Ok(())
            }
            Err(error) => Err(error),
        })
    }
}

impl<C: ControlBackend + 'static> Drop for DeferredShutdownGuard<C> {
    fn drop(&mut self) {
        #[cfg(not(target_arch = "wasm32"))]
        if self.armed {
            self.armed = false;
            if shutdown_hardware(&self.direct).wait().is_ok() {
                finish_deferred_shutdown(&self.shared);
            }
        }
        #[cfg(target_arch = "wasm32")]
        if self.armed {
            self.armed = false;
            let direct = self.direct.stream_handle();
            let shared = Arc::clone(&self.shared);
            wasm_bindgen_futures::spawn_local(async move {
                if shutdown_hardware(&direct).await.is_ok() {
                    finish_deferred_shutdown(&shared);
                }
            });
        }
    }
}

impl<C: ControlBackend> DeviceInner<C> {
    /// Return immutable device metadata.
    pub(crate) fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn stream_handle(&self) -> Self {
        Self {
            direct: self.direct.stream_handle(),
            info: self.info.clone(),
            #[cfg(not(target_arch = "wasm32"))]
            shutdown_on_drop: false,
        }
    }
}

impl<C> DeviceInner<C>
where
    C: ControlBackend,
{
    fn shutdown_operation(&self) -> impl MaybeFuture<Output = Result<()>> + use<C> {
        shutdown_hardware(&self.direct)
    }

    fn shutdown(&mut self) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        #[cfg(not(target_arch = "wasm32"))]
        {
            let operation = self.shutdown_operation();
            let shutdown_on_drop = &mut self.shutdown_on_drop;
            operation.map(move |result| {
                if result.is_ok() {
                    *shutdown_on_drop = false;
                }
                result
            })
        }
        #[cfg(target_arch = "wasm32")]
        {
            self.shutdown_operation()
        }
    }

    /// Apply a high-level receiver configuration through the direct layer.
    pub(crate) fn configure<M: SampleMode>(
        &mut self,
        config: &Config<M>,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C, M> {
        self.direct.configure(config)
    }

    fn sample_rates(&self) -> Vec<u32> {
        self.direct.visible_sample_rates()
    }

    fn set_frequency_hz(
        &mut self,
        frequency_hz: u64,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let operation = self.direct.set_freq(frequency_hz);
        ready(validate_frequency(frequency_hz)).and_then(move |()| operation)
    }

    fn set_sample_rate_hz(
        &mut self,
        sample_rate_hz: u32,
        sample_format: SampleFormat,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let validation = validate_sample_rate(sample_rate_hz, sample_format);
        let operation = self.direct.set_samplerate(sample_rate_hz);
        ready(validation).and_then(move |()| operation)
    }

    fn set_rf_port(&mut self, port: RfPort) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        self.direct.set_rf_port(port)
    }

    fn set_gain(&mut self, gain: GainConfig) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let operation = self.direct.set_gain_config(gain);
        ready(validate_gain(gain)).and_then(move |()| operation)
    }

    fn set_bias_tee(
        &mut self,
        enabled: bool,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        self.direct.set_rf_bias(enabled)
    }

    fn set_packing(&mut self, enabled: bool) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        self.direct.set_packing(enabled)
    }

    fn set_decimation_policy(
        &mut self,
        sample_rate_hz: u32,
        policy: crate::DecimationPolicy,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        self.direct.set_decimation_policy(sample_rate_hz, policy)
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl<C: ControlBackend> Drop for DeviceInner<C> {
    fn drop(&mut self) {
        if self.shutdown_on_drop {
            self.shutdown_on_drop = false;
            let _ = self.shutdown_operation().wait();
        }
    }
}

/// Builder that selects, opens, and initially configures a high-level `nusb` device.
///
/// The generic mode defaults to [`F32Iq`].
///
/// Use [`DeviceBuilder::config`] to validate the same high-level settings without
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
    /// Select a device by parsed 64-bit serial number.
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
        let selector = self.selector;
        ready(self.config.build()).and_then(move |config| {
            let open = match selector {
                DeviceSelector::First => Either::left(HydraSdr::open()),
                DeviceSelector::Serial(serial) => Either::right(HydraSdr::open_sn(serial)),
            };
            open.and_then(move |direct| {
                let cleanup = OpenCleanupGuard::new(&direct);
                let saved_config = config.clone();
                let setup = direct
                    .into_device_info()
                    .map_err(|error| error.at("reading HydraSDR device metadata"))
                    .and_then(move |(direct, info)| {
                        direct
                            .into_configured(config)
                            .map_err(|error| error.at("applying initial HydraSDR configuration"))
                            .map_ok(move |direct| (direct, info))
                    });
                setup.continue_with(move |result| match result {
                    Ok((direct, info)) => {
                        cleanup.disarm();
                        Either::left(ready(Ok(Device {
                            inner: DeviceInner {
                                direct,
                                info,
                                #[cfg(not(target_arch = "wasm32"))]
                                shutdown_on_drop: true,
                            },
                            config: saved_config,
                            shared: Arc::new(Mutex::new(SharedDeviceState::default())),
                        })))
                    }
                    Err(error) => Either::right(cleanup.cleanup().map(move |_| Err(error))),
                })
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

/// Borrowed high-level view of one raw receive block.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SampleBlock<'a> {
    raw: &'a [u8],
    sample_count: usize,
    dropped_samples: u64,
}

impl<'a> SampleBlock<'a> {
    pub(crate) const fn new(raw: &'a [u8], sample_count: usize, dropped_samples: u64) -> Self {
        Self {
            raw,
            sample_count,
            dropped_samples,
        }
    }

    fn from_transfer(transfer: Transfer<'a>) -> Self {
        Self::new(
            transfer.samples,
            transfer.sample_count,
            transfer.dropped_samples,
        )
    }

    /// Raw USB bytes for this sample block.
    pub const fn raw_bytes(&self) -> &'a [u8] {
        self.raw
    }

    /// Sample count reported for this block.
    pub const fn sample_count(&self) -> usize {
        self.sample_count
    }

    /// Estimated sample count represented by USB buffers the driver discarded.
    ///
    /// RFOne transfers have no sequence numbers, so this cannot include samples
    /// lost inside the device while the host transfer queue was exhausted.
    pub const fn dropped_samples(&self) -> u64 {
        self.dropped_samples
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ReceiverState {
    Stopped,
    CleanupRequired,
    Running,
}

/// Owned synchronous raw ADC stream state used by the typed stream facade.
#[cfg(not(target_arch = "wasm32"))]
struct OwnedRawRxStreamInner<C: ControlBackend + StreamingBackend> {
    device: Option<DeviceInner<C>>,
    stream: Option<DirectRawRxStream<C::BulkIn>>,
    stats: StreamingStats,
    state: ReceiverState,
}

#[cfg(not(target_arch = "wasm32"))]
impl<C> OwnedRawRxStreamInner<C>
where
    C: ControlBackend + StreamingBackend,
{
    fn new(device: DeviceInner<C>) -> Self {
        Self {
            device: Some(device),
            stream: None,
            stats: StreamingStats::default(),
            state: ReceiverState::Stopped,
        }
    }

    fn start(&mut self) -> Result<()> {
        match self.state {
            ReceiverState::Running => return Ok(()),
            ReceiverState::CleanupRequired => return Err(Error::Busy),
            ReceiverState::Stopped => {}
        }
        let device = self
            .device
            .as_mut()
            .expect("owned raw stream retains its device");
        self.state = ReceiverState::CleanupRequired;
        self.stream = Some(device.direct.start_raw_rx_stream()?);
        self.state = ReceiverState::Running;
        Ok(())
    }

    fn next_block(&mut self, timeout: Duration) -> Result<Option<SampleBlock<'_>>> {
        if self.state != ReceiverState::Running {
            return Err(Error::stream_closed("raw RX stream is stopped"));
        }
        let desired_packing = self
            .device
            .as_ref()
            .expect("owned raw stream retains its control handle")
            .direct
            .streaming_packing_enabled();
        let packing_changed = self
            .stream
            .as_ref()
            .is_some_and(|stream| stream.packing_enabled() != desired_packing);
        if packing_changed {
            let device = self
                .device
                .as_mut()
                .expect("owned raw stream retains its control handle");
            self.state = ReceiverState::CleanupRequired;
            let stream = self.stream.take().expect("running raw stream exists");
            let (stats, result) = device.direct.close_raw_rx_stream(stream);
            self.stats.accumulate(stats);
            result?;
            self.stream = Some(device.direct.start_raw_rx_stream()?);
            self.state = ReceiverState::Running;
        }
        let stream = self
            .stream
            .as_mut()
            .ok_or(Error::stream_closed("raw RX stream is closed"))?;
        match stream.next_transfer(timeout) {
            Ok(transfer) => Ok(transfer.map(SampleBlock::from_transfer)),
            Err(error) => {
                self.state = ReceiverState::CleanupRequired;
                Err(error)
            }
        }
    }

    fn stop(&mut self) -> Result<StreamingStats> {
        if self.state == ReceiverState::Stopped {
            return Ok(self.stats);
        }
        let device = self
            .device
            .as_mut()
            .expect("owned raw stream retains its device");
        let result = if let Some(stream) = self.stream.take() {
            let (stats, result) = device.direct.close_raw_rx_stream(stream);
            self.stats.accumulate(stats);
            result
        } else {
            device.direct.receiver_mode(ReceiverMode::Off).wait()
        };
        match result {
            Ok(()) => {
                self.state = ReceiverState::Stopped;
                Ok(self.stats)
            }
            Err(error) => {
                self.state = ReceiverState::CleanupRequired;
                Err(error)
            }
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl<C> Drop for OwnedRawRxStreamInner<C>
where
    C: ControlBackend + StreamingBackend,
{
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

/// Owned synchronous converted `F32Iq` stream state.
#[cfg(not(target_arch = "wasm32"))]
struct F32RxStreamInner<C: ControlBackend + StreamingBackend> {
    device: Option<DeviceInner<C>>,
    stream: Option<DirectRxStream<C::BulkIn>>,
    stats: StreamingStats,
    state: ReceiverState,
}

#[cfg(not(target_arch = "wasm32"))]
impl<C> F32RxStreamInner<C>
where
    C: ControlBackend + StreamingBackend,
{
    fn new(device: DeviceInner<C>) -> Self {
        Self {
            device: Some(device),
            stream: None,
            stats: StreamingStats::default(),
            state: ReceiverState::Stopped,
        }
    }

    fn start(&mut self) -> Result<()> {
        match self.state {
            ReceiverState::Running => return Ok(()),
            ReceiverState::CleanupRequired => return Err(Error::Busy),
            ReceiverState::Stopped => {}
        }
        let device = self
            .device
            .as_mut()
            .expect("owned synchronous stream retains its device");
        self.state = ReceiverState::CleanupRequired;
        self.stream = Some(device.direct.start_rx_stream()?);
        self.state = ReceiverState::Running;
        Ok(())
    }

    /// Read converted complex samples into `out`.
    fn read(&mut self, out: &mut [Complex32], timeout: Duration) -> Result<usize> {
        if self.state != ReceiverState::Running {
            return Err(Error::stream_closed("synchronous F32 RX stream is stopped"));
        }
        let factor = self
            .device
            .as_ref()
            .expect("owned synchronous stream retains its control handle")
            .direct
            .streaming_decimation_factor();
        let stream = self
            .stream
            .as_mut()
            .ok_or(Error::stream_closed("F32 RX stream is closed"))?;
        stream.set_decimation_factor(factor)?;
        let result = stream.read_float32_iq(out, timeout);
        if result.is_err() {
            self.state = ReceiverState::CleanupRequired;
        }
        result
    }

    /// Request receiver-off cleanup. Repeated calls are no-ops.
    fn stop(&mut self) -> Result<StreamingStats> {
        if self.state == ReceiverState::Stopped {
            return Ok(self.stats);
        }
        let device = self
            .device
            .as_mut()
            .expect("owned synchronous stream retains its device");
        let result = if let Some(stream) = self.stream.take() {
            let (stats, result) = device.direct.close_rx_stream(stream);
            self.stats.accumulate(stats);
            result
        } else {
            device.direct.receiver_mode(ReceiverMode::Off).wait()
        };
        match result {
            Ok(()) => {
                self.state = ReceiverState::Stopped;
                Ok(self.stats)
            }
            Err(error) => {
                self.state = ReceiverState::CleanupRequired;
                Err(error)
            }
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl<C> Drop for F32RxStreamInner<C>
where
    C: ControlBackend + StreamingBackend,
{
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

/// Owned async stream state for raw ADC blocks.
struct AsyncRawRxStreamInner<C: ControlBackend + AsyncStreamingBackend> {
    device: Option<DeviceInner<C>>,
    stream: Option<DirectAsyncRawRxStream<C::BulkIn>>,
    stats: StreamingStats,
    state: ReceiverState,
}

impl<C> AsyncRawRxStreamInner<C>
where
    C: ControlBackend + AsyncStreamingBackend,
{
    fn new(device: DeviceInner<C>) -> Self {
        Self {
            device: Some(device),
            stream: None,
            stats: StreamingStats::default(),
            state: ReceiverState::Stopped,
        }
    }

    fn current_stats(&self) -> StreamingStats {
        self.stream
            .as_ref()
            .map_or(self.stats, |stream| self.stats.combined(stream.stats()))
    }

    fn retire_stream(&mut self) {
        if let Some(mut stream) = self.stream.take() {
            let stats = stream.close();
            self.stats.accumulate(stats);
        }
    }

    async fn start(&mut self) -> Result<()> {
        match self.state {
            ReceiverState::Running => return Ok(()),
            ReceiverState::CleanupRequired => return Err(Error::Busy),
            ReceiverState::Stopped => {}
        }
        let desired_packing = self
            .device
            .as_ref()
            .expect("owned async stream retains its device")
            .direct
            .streaming_packing_enabled();
        let stream_reusable = self.stream.as_ref().is_some_and(|stream| {
            !stream.is_closed() && stream.packing_enabled() == desired_packing
        });
        if !stream_reusable {
            self.retire_stream();
        }
        let device = self
            .device
            .as_mut()
            .expect("owned async stream retains its device");
        self.state = ReceiverState::CleanupRequired;
        if stream_reusable {
            device.direct.receiver_mode(ReceiverMode::Rx).await?;
            self.state = ReceiverState::Running;
            return Ok(());
        }
        let stream = device.direct.start_raw_rx_stream_async().await?;
        self.stream = Some(stream);
        self.state = ReceiverState::Running;
        Ok(())
    }

    /// Read the next sample block.
    async fn next_block(&mut self) -> Result<Option<SampleBlock<'_>>> {
        if self.state != ReceiverState::Running {
            return Err(Error::stream_closed("async raw RX stream is stopped"));
        }
        let desired_packing = self
            .device
            .as_ref()
            .expect("owned async stream retains its control handle")
            .direct
            .streaming_packing_enabled();
        let packing_changed = self
            .stream
            .as_ref()
            .is_some_and(|stream| stream.packing_enabled() != desired_packing);
        if packing_changed {
            self.retire_stream();
            let device = self
                .device
                .as_mut()
                .expect("owned async stream retains its control handle");
            self.state = ReceiverState::CleanupRequired;
            self.stream = Some(device.direct.start_raw_rx_stream_async().await?);
            self.state = ReceiverState::Running;
        }
        let stream = self
            .stream
            .as_mut()
            .ok_or(Error::stream_closed("async RX stream is closed"))?;
        match stream.next_transfer().await {
            Ok(transfer) => Ok(transfer.map(SampleBlock::from_transfer)),
            Err(error) => {
                self.state = ReceiverState::CleanupRequired;
                Err(error)
            }
        }
    }

    async fn stop(&mut self) -> Result<StreamingStats> {
        if self.state == ReceiverState::Stopped {
            return Ok(self.current_stats());
        }
        self.device
            .as_ref()
            .expect("owned async stream retains its device")
            .direct
            .receiver_mode(ReceiverMode::Off)
            .await?;
        self.state = ReceiverState::Stopped;
        if let Some(stream) = self.stream.as_mut() {
            stream.pause()?;
        }
        Ok(self.current_stats())
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn stop_on_drop(&mut self) -> Result<StreamingStats> {
        if self.state == ReceiverState::Stopped {
            return Ok(self.current_stats());
        }
        self.device
            .as_ref()
            .expect("owned async stream retains its device")
            .direct
            .receiver_mode(ReceiverMode::Off)
            .wait()?;
        self.state = ReceiverState::Stopped;
        Ok(self.current_stats())
    }
}

impl<C> Drop for AsyncRawRxStreamInner<C>
where
    C: ControlBackend + AsyncStreamingBackend,
{
    fn drop(&mut self) {
        #[cfg(not(target_arch = "wasm32"))]
        let _ = self.stop_on_drop();
        self.retire_stream();
    }
}

/// Owned async stream state for converted `F32Iq` samples.
struct AsyncF32RxStreamInner<C: ControlBackend + AsyncStreamingBackend> {
    device: Option<DeviceInner<C>>,
    stream: Option<AsyncDirectRxStream<C::BulkIn>>,
    stats: StreamingStats,
    state: ReceiverState,
}

impl<C> AsyncF32RxStreamInner<C>
where
    C: ControlBackend + AsyncStreamingBackend,
{
    fn new(device: DeviceInner<C>) -> Self {
        Self {
            device: Some(device),
            stream: None,
            stats: StreamingStats::default(),
            state: ReceiverState::Stopped,
        }
    }

    fn current_stats(&self) -> StreamingStats {
        self.stream
            .as_ref()
            .map_or(self.stats, |stream| self.stats.combined(stream.stats()))
    }

    fn retire_stream(&mut self) {
        if let Some(mut stream) = self.stream.take() {
            let stats = stream.close();
            self.stats.accumulate(stats);
        }
    }

    async fn start(&mut self) -> Result<()> {
        match self.state {
            ReceiverState::Running => return Ok(()),
            ReceiverState::CleanupRequired => return Err(Error::Busy),
            ReceiverState::Stopped => {}
        }
        let device = self
            .device
            .as_mut()
            .expect("owned async stream retains its device");
        if let Some(stream) = self.stream.as_mut() {
            stream.set_decimation_factor(device.direct.streaming_decimation_factor())?;
            self.state = ReceiverState::CleanupRequired;
            device.direct.receiver_mode(ReceiverMode::Rx).await?;
            self.state = ReceiverState::Running;
            return Ok(());
        }
        self.state = ReceiverState::CleanupRequired;
        let stream = device.direct.start_rx_stream_async().await?;
        self.stream = Some(stream);
        self.state = ReceiverState::Running;
        Ok(())
    }

    /// Read converted complex samples into `out`.
    async fn read(&mut self, out: &mut [Complex32]) -> Result<usize> {
        if self.state != ReceiverState::Running {
            return Err(Error::stream_closed("async F32 RX stream is stopped"));
        }
        let factor = self
            .device
            .as_ref()
            .expect("owned async stream retains its control handle")
            .direct
            .streaming_decimation_factor();
        let stream = self
            .stream
            .as_mut()
            .ok_or(Error::stream_closed("async F32 RX stream is closed"))?;
        stream.set_decimation_factor(factor)?;
        let result = stream.read_float32_iq(out).await;
        match result {
            Ok(written) => Ok(written),
            Err(error) => {
                self.retire_stream();
                self.state = ReceiverState::CleanupRequired;
                Err(error)
            }
        }
    }

    async fn stop(&mut self) -> Result<StreamingStats> {
        if self.state == ReceiverState::Stopped {
            return Ok(self.current_stats());
        }
        self.device
            .as_ref()
            .expect("owned async stream retains its device")
            .direct
            .receiver_mode(ReceiverMode::Off)
            .await?;
        self.state = ReceiverState::Stopped;
        if let Some(stream) = self.stream.as_mut() {
            stream.pause()?;
        }
        Ok(self.current_stats())
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn stop_on_drop(&mut self) -> Result<StreamingStats> {
        if self.state == ReceiverState::Stopped {
            return Ok(self.current_stats());
        }
        self.device
            .as_ref()
            .expect("owned async stream retains its device")
            .direct
            .receiver_mode(ReceiverMode::Off)
            .wait()?;
        self.state = ReceiverState::Stopped;
        Ok(self.current_stats())
    }
}

impl<C> Drop for AsyncF32RxStreamInner<C>
where
    C: ControlBackend + AsyncStreamingBackend,
{
    fn drop(&mut self) {
        #[cfg(not(target_arch = "wasm32"))]
        let _ = self.stop_on_drop();
        self.retire_stream();
    }
}

enum RxStreamState {
    Dormant(DeviceInner<NusbControl>),
    Poisoned,
    #[cfg(not(target_arch = "wasm32"))]
    BlockingRaw(OwnedRawRxStreamInner<NusbControl>),
    #[cfg(not(target_arch = "wasm32"))]
    BlockingF32(Box<F32RxStreamInner<NusbControl>>),
    AsyncRaw(AsyncRawRxStreamInner<NusbControl>),
    AsyncF32(Box<AsyncF32RxStreamInner<NusbControl>>),
}

#[derive(Debug)]
struct RxStreamClaim {
    shared: SharedState,
}

impl RxStreamClaim {
    fn acquire(shared: &SharedState) -> Result<Self> {
        let mut state = lock_shared(shared);
        if state.device != DeviceLifecycle::Open {
            return Err(Error::DeviceClosed);
        }
        if state.stream_claimed || state.receiver != ReceiverSlot::Idle {
            return Err(Error::Busy);
        }
        state.stream_claimed = true;
        Ok(Self {
            shared: Arc::clone(shared),
        })
    }
}

impl Drop for RxStreamClaim {
    fn drop(&mut self) {
        lock_shared(&self.shared).stream_claimed = false;
    }
}

#[derive(Debug)]
struct ReceiverLease<C: ControlBackend + 'static = NusbControl> {
    shared: SharedState,
    cleanup: Option<HydraSdr<C>>,
    armed: bool,
}

impl<C: ControlBackend + 'static> ReceiverLease<C> {
    fn acquire(shared: &SharedState, cleanup: HydraSdr<C>) -> Result<Self> {
        let mut state = lock_shared(shared);
        if state.device != DeviceLifecycle::Open {
            return Err(Error::DeviceClosed);
        }
        if state.receiver != ReceiverSlot::Idle {
            return Err(Error::Busy);
        }
        state.receiver = ReceiverSlot::Held;
        Ok(Self {
            shared: Arc::clone(shared),
            cleanup: Some(cleanup),
            armed: true,
        })
    }

    fn release(mut self) -> Option<DeferredShutdownGuard<C>> {
        let deferred = {
            let mut state = lock_shared(&self.shared);
            if state.receiver == ReceiverSlot::Held {
                state.receiver = ReceiverSlot::Idle;
            }
            state.device == DeviceLifecycle::DropCleanupPending
        };
        self.armed = false;
        deferred.then(|| {
            DeferredShutdownGuard::new(
                self.cleanup
                    .take()
                    .expect("armed receiver lease owns cleanup"),
                &self.shared,
            )
        })
    }
}

impl<C: ControlBackend + 'static> Drop for ReceiverLease<C> {
    fn drop(&mut self) {
        // Never infer that hardware is off merely because its owner disappeared.
        if self.armed {
            let deferred = {
                let mut state = lock_shared(&self.shared);
                if state.receiver == ReceiverSlot::Held {
                    state.receiver = ReceiverSlot::Orphaned;
                }
                state.device == DeviceLifecycle::DropCleanupPending
            };
            self.armed = false;
            if deferred {
                let cleanup = DeferredShutdownGuard::new(
                    self.cleanup
                        .take()
                        .expect("armed receiver lease owns cleanup"),
                    &self.shared,
                );
                drop(cleanup);
            }
        }
    }
}

/// Owned receive stream for sample mode `M`, defaulting to [`F32Iq`].
///
/// Create this stream with [`Device::rx_stream`]. Waiting the first
/// [`RxStream::start`] operation selects blocking USB on native targets;
/// awaiting it selects asynchronous USB. Do not mix waiting and awaiting on
/// one stream.
#[must_use = "RX streams retain the device's exclusive stream claim until dropped"]
pub struct RxStream<M: SampleMode = F32Iq> {
    state: RxStreamState,
    shared: SharedState,
    receiver: Option<ReceiverLease<NusbControl>>,
    _claim: RxStreamClaim,
    mode: PhantomData<fn() -> M>,
}

impl<M: SampleMode> RxStream<M> {
    fn new(device: DeviceInner<NusbControl>, shared: SharedState, claim: RxStreamClaim) -> Self {
        Self {
            state: RxStreamState::Dormant(device),
            shared,
            receiver: None,
            _claim: claim,
            mode: PhantomData,
        }
    }

    /// Start reception and the persistent USB transfer queue.
    ///
    /// Returns [`Error::DeviceClosed`] after the owning device has begun
    /// shutdown. A stream requiring cleanup must be stopped before restarting.
    ///
    /// Call [`MaybeFuture::wait`] for blocking operation on native targets, or
    /// await the returned operation for asynchronous operation.
    pub fn start(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        StartOperation { stream: self }
    }

    /// Stop reception and return accumulated streaming counters.
    pub fn stop(&mut self) -> impl MaybeFuture<Output = Result<StreamingStats>> + '_ {
        StopOperation { stream: self }
    }

    fn initialize_async(&mut self) -> Result<()> {
        if matches!(
            self.state,
            RxStreamState::AsyncRaw(_) | RxStreamState::AsyncF32(_)
        ) {
            return Ok(());
        }
        #[cfg(not(target_arch = "wasm32"))]
        if matches!(
            self.state,
            RxStreamState::BlockingRaw(_) | RxStreamState::BlockingF32(_)
        ) {
            return Err(Error::Busy);
        }
        let state = core::mem::replace(&mut self.state, RxStreamState::Poisoned);
        let RxStreamState::Dormant(device) = state else {
            self.state = state;
            return Err(Error::stream_closed("RX stream has no device"));
        };
        self.state = match M::FORMAT {
            SampleFormat::RawAdc => RxStreamState::AsyncRaw(AsyncRawRxStreamInner::new(device)),
            SampleFormat::F32Iq => {
                RxStreamState::AsyncF32(Box::new(AsyncF32RxStreamInner::new(device)))
            }
        };
        Ok(())
    }

    async fn start_async(&mut self) -> Result<()> {
        self.initialize_async()?;
        if !self.begin_start()? {
            return Ok(());
        }
        match &mut self.state {
            RxStreamState::AsyncRaw(stream) => stream.start().await,
            RxStreamState::AsyncF32(stream) => stream.start().await,
            _ => Err(Error::Busy),
        }
    }

    async fn stop_async(&mut self) -> Result<StreamingStats> {
        let result = match &mut self.state {
            RxStreamState::Dormant(_) => Ok(StreamingStats::default()),
            RxStreamState::AsyncRaw(stream) => stream.stop().await,
            RxStreamState::AsyncF32(stream) => stream.stop().await,
            #[cfg(not(target_arch = "wasm32"))]
            RxStreamState::BlockingRaw(_) | RxStreamState::BlockingF32(_) => Err(Error::Busy),
            RxStreamState::Poisoned => Err(Error::stream_closed("RX stream has no device")),
        };
        let (stats, cleanup) = self.finish_stop(result)?;
        if let Some(cleanup) = cleanup {
            cleanup.cleanup().await?;
        }
        Ok(stats)
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn initialize_blocking(&mut self) -> Result<()> {
        if matches!(
            self.state,
            RxStreamState::BlockingRaw(_) | RxStreamState::BlockingF32(_)
        ) {
            return Ok(());
        }
        if matches!(
            self.state,
            RxStreamState::AsyncRaw(_) | RxStreamState::AsyncF32(_)
        ) {
            return Err(Error::Busy);
        }
        let state = core::mem::replace(&mut self.state, RxStreamState::Poisoned);
        let RxStreamState::Dormant(device) = state else {
            self.state = state;
            return Err(Error::stream_closed("RX stream has no device"));
        };
        self.state = match M::FORMAT {
            SampleFormat::RawAdc => RxStreamState::BlockingRaw(OwnedRawRxStreamInner::new(device)),
            SampleFormat::F32Iq => {
                RxStreamState::BlockingF32(Box::new(F32RxStreamInner::new(device)))
            }
        };
        Ok(())
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn start_blocking(&mut self) -> Result<()> {
        self.initialize_blocking()?;
        if !self.begin_start()? {
            return Ok(());
        }
        match &mut self.state {
            RxStreamState::BlockingRaw(stream) => stream.start(),
            RxStreamState::BlockingF32(stream) => stream.start(),
            _ => Err(Error::Busy),
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn stop_blocking(&mut self) -> Result<StreamingStats> {
        let result = match &mut self.state {
            RxStreamState::Dormant(_) => Ok(StreamingStats::default()),
            RxStreamState::BlockingRaw(stream) => stream.stop(),
            RxStreamState::BlockingF32(stream) => stream.stop(),
            RxStreamState::AsyncRaw(_) | RxStreamState::AsyncF32(_) => Err(Error::Busy),
            RxStreamState::Poisoned => Err(Error::stream_closed("RX stream has no device")),
        };
        let (stats, cleanup) = self.finish_stop(result)?;
        if let Some(cleanup) = cleanup {
            cleanup.cleanup().wait()?;
        }
        Ok(stats)
    }

    fn receiver_state(&self) -> ReceiverState {
        match &self.state {
            RxStreamState::Dormant(_) => ReceiverState::Stopped,
            RxStreamState::Poisoned => ReceiverState::CleanupRequired,
            #[cfg(not(target_arch = "wasm32"))]
            RxStreamState::BlockingRaw(stream) => stream.state,
            #[cfg(not(target_arch = "wasm32"))]
            RxStreamState::BlockingF32(stream) => stream.state,
            RxStreamState::AsyncRaw(stream) => stream.state,
            RxStreamState::AsyncF32(stream) => stream.state,
        }
    }

    fn begin_start(&mut self) -> Result<bool> {
        ensure_device_open(&self.shared)?;
        match (self.receiver.is_some(), self.receiver_state()) {
            (true, ReceiverState::Running) => Ok(false),
            (true, ReceiverState::Stopped | ReceiverState::CleanupRequired)
            | (false, ReceiverState::Running | ReceiverState::CleanupRequired) => Err(Error::Busy),
            (false, ReceiverState::Stopped) => {
                let cleanup = self
                    .control_handle()
                    .ok_or(Error::stream_closed("RX stream has no device"))?;
                self.receiver = Some(ReceiverLease::acquire(&self.shared, cleanup)?);
                Ok(true)
            }
        }
    }

    fn finish_stop<T>(
        &mut self,
        result: Result<T>,
    ) -> Result<(T, Option<DeferredShutdownGuard<NusbControl>>)> {
        let value = result?;
        let cleanup = self.receiver.take().and_then(ReceiverLease::release);
        Ok((value, cleanup))
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn cleanup_on_drop(&mut self) -> Result<()> {
        match &mut self.state {
            RxStreamState::Dormant(_) => Ok(()),
            RxStreamState::BlockingRaw(stream) => stream.stop().map(|_| ()),
            RxStreamState::BlockingF32(stream) => stream.stop().map(|_| ()),
            RxStreamState::AsyncRaw(stream) => stream.stop_on_drop().map(|_| ()),
            RxStreamState::AsyncF32(stream) => stream.stop_on_drop().map(|_| ()),
            RxStreamState::Poisoned => Err(Error::stream_closed("RX stream has no device")),
        }
    }

    fn control_handle(&self) -> Option<HydraSdr<NusbControl>> {
        let device = match &self.state {
            RxStreamState::Dormant(device) => device,
            #[cfg(not(target_arch = "wasm32"))]
            RxStreamState::BlockingRaw(stream) => stream.device.as_ref()?,
            #[cfg(not(target_arch = "wasm32"))]
            RxStreamState::BlockingF32(stream) => stream.device.as_ref()?,
            RxStreamState::AsyncRaw(stream) => stream.device.as_ref()?,
            RxStreamState::AsyncF32(stream) => stream.device.as_ref()?,
            RxStreamState::Poisoned => return None,
        };
        Some(device.direct.stream_handle())
    }
}

impl<M: SampleMode> Drop for RxStream<M> {
    fn drop(&mut self) {
        #[cfg(not(target_arch = "wasm32"))]
        if self.receiver.is_some() && self.cleanup_on_drop().is_ok() {
            let cleanup = self
                .receiver
                .take()
                .expect("receiver lease checked above")
                .release();
            if let Some(cleanup) = cleanup {
                let _ = cleanup.cleanup().wait();
            }
        }
        #[cfg(target_arch = "wasm32")]
        if let Some(direct) = self.control_handle()
            && let Some(receiver) = self.receiver.take()
        {
            wasm_bindgen_futures::spawn_local(async move {
                if direct.receiver_mode(ReceiverMode::Off).await.is_ok() {
                    if let Some(cleanup) = receiver.release() {
                        let _ = cleanup.cleanup().await;
                    }
                }
            });
        }
    }
}

impl RxStream<RawAdc> {
    /// Read the next zero-copy raw ADC USB block.
    ///
    /// The returned block borrows one buffer from the fixed transfer pool. Its
    /// buffer is resubmitted on the next call. For blocking operation, `timeout`
    /// bounds the wait and [`None`] waits indefinitely. The timeout is ignored
    /// when this operation is awaited, which waits for the next USB completion.
    pub fn next_block(
        &mut self,
        timeout: Option<Duration>,
    ) -> impl MaybeFuture<Output = Result<Option<SampleBlock<'_>>>> + '_ {
        NextBlockOperation {
            stream: self,
            timeout,
        }
    }

    async fn next_block_async(&mut self) -> Result<Option<SampleBlock<'_>>> {
        match &mut self.state {
            RxStreamState::AsyncRaw(stream) => stream.next_block().await,
            _ => Err(Error::stream_closed(
                "raw RX stream is not running asynchronously",
            )),
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn next_block_blocking(&mut self, timeout: Duration) -> Result<Option<SampleBlock<'_>>> {
        match &mut self.state {
            RxStreamState::BlockingRaw(stream) => stream.next_block(timeout),
            _ => Err(Error::stream_closed(
                "raw RX stream is not running in blocking mode",
            )),
        }
    }
}

impl RxStream<F32Iq> {
    /// Convert samples directly into the caller-provided complex output slice.
    ///
    /// Blocking operation fills the slice until its total `timeout` expires;
    /// [`None`] waits indefinitely. Asynchronous operation drains buffered data
    /// or waits for at most one new USB completion, so it may return fewer
    /// samples than the slice can hold; the timeout is ignored when this
    /// operation is awaited.
    pub fn read<'a>(
        &'a mut self,
        out: &'a mut [Complex32],
        timeout: Option<Duration>,
    ) -> impl MaybeFuture<Output = Result<usize>> + 'a {
        ReadOperation {
            stream: self,
            out,
            timeout,
        }
    }

    async fn read_async(&mut self, out: &mut [Complex32]) -> Result<usize> {
        match &mut self.state {
            RxStreamState::AsyncF32(stream) => stream.read(out).await,
            _ => Err(Error::stream_closed(
                "F32 IQ stream is not running asynchronously",
            )),
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn read_blocking(&mut self, out: &mut [Complex32], timeout: Duration) -> Result<usize> {
        match &mut self.state {
            RxStreamState::BlockingF32(stream) => stream.read(out, timeout),
            _ => Err(Error::stream_closed(
                "F32 IQ stream is not running in blocking mode",
            )),
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
type OperationFuture<'a, T> = Pin<Box<dyn Future<Output = T> + Send + 'a>>;
#[cfg(target_arch = "wasm32")]
type OperationFuture<'a, T> = Pin<Box<dyn Future<Output = T> + 'a>>;

struct StartOperation<'a, M: SampleMode> {
    stream: &'a mut RxStream<M>,
}

impl<'a, M: SampleMode> IntoFuture for StartOperation<'a, M> {
    type Output = Result<()>;
    type IntoFuture = OperationFuture<'a, Self::Output>;

    fn into_future(self) -> Self::IntoFuture {
        Box::pin(self.stream.start_async())
    }
}

impl<M: SampleMode> MaybeFuture for StartOperation<'_, M> {
    #[cfg(not(target_arch = "wasm32"))]
    fn wait(self) -> Self::Output {
        self.stream.start_blocking()
    }
}

struct StopOperation<'a, M: SampleMode> {
    stream: &'a mut RxStream<M>,
}

impl<'a, M: SampleMode> IntoFuture for StopOperation<'a, M> {
    type Output = Result<StreamingStats>;
    type IntoFuture = OperationFuture<'a, Self::Output>;

    fn into_future(self) -> Self::IntoFuture {
        Box::pin(self.stream.stop_async())
    }
}

impl<M: SampleMode> MaybeFuture for StopOperation<'_, M> {
    #[cfg(not(target_arch = "wasm32"))]
    fn wait(self) -> Self::Output {
        self.stream.stop_blocking()
    }
}

struct NextBlockOperation<'a> {
    stream: &'a mut RxStream<RawAdc>,
    timeout: Option<Duration>,
}

impl<'a> IntoFuture for NextBlockOperation<'a> {
    type Output = Result<Option<SampleBlock<'a>>>;
    type IntoFuture = OperationFuture<'a, Self::Output>;

    fn into_future(self) -> Self::IntoFuture {
        let _ = self.timeout;
        Box::pin(self.stream.next_block_async())
    }
}

impl MaybeFuture for NextBlockOperation<'_> {
    #[cfg(not(target_arch = "wasm32"))]
    fn wait(self) -> Self::Output {
        self.stream
            .next_block_blocking(self.timeout.unwrap_or(Duration::MAX))
    }
}

struct ReadOperation<'a> {
    stream: &'a mut RxStream<F32Iq>,
    out: &'a mut [Complex32],
    timeout: Option<Duration>,
}

impl<'a> IntoFuture for ReadOperation<'a> {
    type Output = Result<usize>;
    type IntoFuture = OperationFuture<'a, Self::Output>;

    fn into_future(self) -> Self::IntoFuture {
        let _ = self.timeout;
        Box::pin(self.stream.read_async(self.out))
    }
}

impl MaybeFuture for ReadOperation<'_> {
    #[cfg(not(target_arch = "wasm32"))]
    fn wait(self) -> Self::Output {
        self.stream
            .read_blocking(self.out, self.timeout.unwrap_or(Duration::MAX))
    }
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    use std::sync::{Arc, Mutex};

    use futures_lite::future::block_on;

    use super::*;
    use crate::streaming::{AsyncBulkInBackend, BulkInCompletion};
    #[cfg(not(target_arch = "wasm32"))]
    use crate::streaming::{BulkInBackend, StreamingBackend};
    use crate::usb::control::VendorControlRequest;

    #[test]
    fn receive_stream_claim_is_exclusive_and_released_on_drop() {
        let shared = Arc::new(Mutex::new(SharedDeviceState::default()));
        let claim = RxStreamClaim::acquire(&shared).expect("acquire first stream claim");

        assert!(
            RxStreamClaim::acquire(&shared)
                .is_err_and(|error| error.kind() == crate::ErrorKind::Busy)
        );
        drop(claim);
        assert!(RxStreamClaim::acquire(&shared).is_ok());
    }

    #[test]
    fn shutdown_rejects_a_held_receiver_lease() {
        let shared = Arc::new(Mutex::new(SharedDeviceState::default()));
        let _receiver =
            ReceiverLease::acquire(&shared, HydraSdr::from_control(FakeControl::default()))
                .expect("acquire receiver lease");

        assert!(begin_shutdown(&shared).is_err_and(|error| error.kind() == crate::ErrorKind::Busy));
        assert_eq!(lock_shared(&shared).device, DeviceLifecycle::Open);
    }

    #[test]
    fn stopped_stream_is_invalidated_when_shutdown_begins() {
        let shared = Arc::new(Mutex::new(SharedDeviceState::default()));
        let _claim = RxStreamClaim::acquire(&shared).expect("claim stopped stream");

        assert!(begin_shutdown(&shared).expect("begin shutdown"));
        assert!(matches!(
            ReceiverLease::acquire(&shared, HydraSdr::from_control(FakeControl::default())),
            Err(Error::DeviceClosed)
        ));
        finish_shutdown(&shared);
        assert!(matches!(
            ReceiverLease::acquire(&shared, HydraSdr::from_control(FakeControl::default())),
            Err(Error::DeviceClosed)
        ));
    }

    #[test]
    fn failed_shutdown_remains_closing_and_can_be_retried() {
        let shared = Arc::new(Mutex::new(SharedDeviceState::default()));

        assert!(begin_shutdown(&shared).expect("begin first shutdown"));
        assert!(complete_shutdown(&shared, Err(Error::Unsupported)).is_err());
        assert_eq!(lock_shared(&shared).device, DeviceLifecycle::Closing);

        assert!(begin_shutdown(&shared).expect("retry shutdown"));
        complete_shutdown(&shared, Ok(())).expect("complete retry");
        assert_eq!(lock_shared(&shared).device, DeviceLifecycle::Closed);
        assert!(!begin_shutdown(&shared).expect("closed shutdown is a no-op"));
    }

    #[test]
    fn closed_device_takes_precedence_over_a_stale_receiver_slot() {
        let shared = Arc::new(Mutex::new(SharedDeviceState {
            device: DeviceLifecycle::Closed,
            receiver: ReceiverSlot::Held,
            stream_claimed: true,
        }));

        assert!(matches!(
            ReceiverLease::acquire(&shared, HydraSdr::from_control(FakeControl::default())),
            Err(Error::DeviceClosed)
        ));
    }

    #[test]
    fn cancelled_start_requires_cleanup_before_shutdown() {
        let shared = Arc::new(Mutex::new(SharedDeviceState::default()));
        let _claim = RxStreamClaim::acquire(&shared).expect("claim stream");
        let receiver =
            ReceiverLease::acquire(&shared, HydraSdr::from_control(FakeControl::default()))
                .expect("reserve receiver for start");

        assert_eq!(lock_shared(&shared).receiver, ReceiverSlot::Held);
        assert!(begin_shutdown(&shared).is_err_and(|error| error.kind() == crate::ErrorKind::Busy));
        assert_eq!(lock_shared(&shared).receiver, ReceiverSlot::Held);
        assert!(begin_shutdown(&shared).is_err_and(|error| error.kind() == crate::ErrorKind::Busy));

        assert!(receiver.release().is_none());
        assert!(begin_shutdown(&shared).expect("begin shutdown after cleanup"));
    }

    #[test]
    fn dropped_receiver_lease_requires_device_cleanup() {
        let shared = Arc::new(Mutex::new(SharedDeviceState::default()));
        let receiver =
            ReceiverLease::acquire(&shared, HydraSdr::from_control(FakeControl::default()))
                .expect("reserve receiver");

        drop(receiver);

        assert_eq!(lock_shared(&shared).receiver, ReceiverSlot::Orphaned);
        assert!(
            RxStreamClaim::acquire(&shared)
                .is_err_and(|error| error.kind() == crate::ErrorKind::Busy)
        );
        assert!(begin_shutdown(&shared).expect("shutdown cleans an orphaned receiver"));
    }

    #[derive(Debug, Default)]
    struct FakeState {
        fail_packing: AtomicBool,
        control_out_count: AtomicUsize,
        control_out_requests: Mutex<Vec<VendorControlRequest>>,
        fail_control_out: AtomicBool,
        fail_control_out_at: AtomicUsize,
        pause_control_out_at: AtomicUsize,
        fail_bulk_completion: AtomicBool,
        bulk_in_count: AtomicUsize,
        cancel_count: AtomicUsize,
    }

    #[derive(Clone, Debug, Default)]
    struct FakeControl {
        state: Arc<FakeState>,
    }

    impl ControlBackend for FakeControl {
        fn control_in(
            &self,
            request: VendorControlRequest,
        ) -> impl MaybeFuture<Output = Result<Vec<u8>>> + use<> {
            let result = if self.state.fail_packing.load(Ordering::SeqCst)
                && request == VendorControlRequest::set_packing(0)
            {
                Err(nusb::transfer::TransferError::Fault.into())
            } else {
                Ok(vec![1])
            };
            ready(result)
        }

        fn control_out(
            &self,
            request: VendorControlRequest,
        ) -> impl MaybeFuture<Output = Result<()>> + use<> {
            FakeControlOut {
                state: Arc::clone(&self.state),
                request,
            }
        }
    }

    struct FakeControlOut {
        state: Arc<FakeState>,
        request: VendorControlRequest,
    }

    impl std::future::IntoFuture for FakeControlOut {
        type Output = Result<()>;
        type IntoFuture = std::pin::Pin<Box<dyn std::future::Future<Output = Result<()>> + Send>>;

        fn into_future(self) -> Self::IntoFuture {
            Box::pin(async move {
                self.state
                    .control_out_requests
                    .lock()
                    .expect("control request lock")
                    .push(self.request);
                let call = self.state.control_out_count.fetch_add(1, Ordering::SeqCst) + 1;
                if self.state.pause_control_out_at.load(Ordering::SeqCst) == call {
                    core::future::pending::<()>().await;
                }
                if self.state.fail_control_out.load(Ordering::SeqCst)
                    || self.state.fail_control_out_at.load(Ordering::SeqCst) == call
                {
                    Err(nusb::transfer::TransferError::Fault.into())
                } else {
                    Ok(())
                }
            })
        }
    }

    impl MaybeFuture for FakeControlOut {
        #[cfg(not(target_arch = "wasm32"))]
        fn wait(self) -> Result<()> {
            self.state
                .control_out_requests
                .lock()
                .expect("control request lock")
                .push(self.request);
            let call = self.state.control_out_count.fetch_add(1, Ordering::SeqCst) + 1;
            if self.state.fail_control_out.load(Ordering::SeqCst)
                || self.state.fail_control_out_at.load(Ordering::SeqCst) == call
            {
                Err(nusb::transfer::TransferError::Fault.into())
            } else {
                Ok(())
            }
        }
    }

    impl AsyncStreamingBackend for FakeControl {
        type BulkIn = FakeAsyncBulkIn;

        fn bulk_in(&self, _endpoint: u8) -> Result<Self::BulkIn> {
            self.state.bulk_in_count.fetch_add(1, Ordering::SeqCst);
            Ok(FakeAsyncBulkIn {
                state: Arc::clone(&self.state),
                submitted: VecDeque::new(),
            })
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    impl StreamingBackend for FakeControl {
        type BulkIn = FakeBulkIn;

        fn bulk_in(&self, _endpoint: u8) -> Result<Self::BulkIn> {
            self.state.bulk_in_count.fetch_add(1, Ordering::SeqCst);
            Ok(FakeBulkIn {
                state: Arc::clone(&self.state),
                submitted: VecDeque::new(),
            })
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[derive(Debug)]
    struct FakeBulkIn {
        state: Arc<FakeState>,
        submitted: VecDeque<Vec<u8>>,
    }

    #[cfg(not(target_arch = "wasm32"))]
    impl BulkInBackend for FakeBulkIn {
        type Buffer = Vec<u8>;

        fn clear_halt(&mut self) -> Result<()> {
            Ok(())
        }

        fn allocate(&self, len: usize) -> Self::Buffer {
            vec![0; len]
        }

        fn submit(&mut self, buffer: Self::Buffer) {
            self.submitted.push_back(buffer);
        }

        fn pending(&self) -> usize {
            self.submitted.len()
        }

        fn wait_next_complete(
            &mut self,
            _timeout: Duration,
        ) -> Option<BulkInCompletion<Self::Buffer>> {
            let buffer = self
                .submitted
                .pop_front()
                .expect("fake synchronous bulk queue contains submitted buffers");
            Some(BulkInCompletion {
                actual_len: buffer.len(),
                buffer,
                status: if self
                    .state
                    .fail_bulk_completion
                    .swap(false, Ordering::SeqCst)
                {
                    Err(nusb::transfer::TransferError::Fault.into())
                } else {
                    Ok(())
                },
            })
        }

        fn cancel_all(&mut self) {
            self.state.cancel_count.fetch_add(1, Ordering::SeqCst);
        }
    }

    #[derive(Debug)]
    struct FakeAsyncBulkIn {
        state: Arc<FakeState>,
        submitted: VecDeque<Vec<u8>>,
    }

    impl AsyncBulkInBackend for FakeAsyncBulkIn {
        type Buffer = Vec<u8>;

        async fn clear_halt_async(&mut self) -> Result<()> {
            Ok(())
        }

        fn allocate(&self, len: usize) -> Self::Buffer {
            vec![0; len]
        }

        fn submit(&mut self, buffer: Self::Buffer) {
            self.submitted.push_back(buffer);
        }

        fn pending(&self) -> usize {
            self.submitted.len()
        }

        async fn next_complete_async(&mut self) -> BulkInCompletion<Self::Buffer> {
            let buffer = self
                .submitted
                .pop_front()
                .expect("fake async bulk queue contains submitted buffers");
            BulkInCompletion {
                actual_len: buffer.len(),
                buffer,
                status: if self
                    .state
                    .fail_bulk_completion
                    .swap(false, Ordering::SeqCst)
                {
                    Err(nusb::transfer::TransferError::Fault.into())
                } else {
                    Ok(())
                },
            }
        }

        fn cancel_all(&mut self) {
            self.state.cancel_count.fetch_add(1, Ordering::SeqCst);
        }
    }

    fn fake_device(control: FakeControl) -> DeviceInner<FakeControl> {
        DeviceInner {
            direct: HydraSdr::from_control(control),
            info: DeviceInfo {
                board_name: "fake HydraSDR",
                firmware_version: "fake firmware".to_owned(),
                serial: None,
                rf_ports: Vec::new(),
            },
            #[cfg(not(target_arch = "wasm32"))]
            shutdown_on_drop: true,
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn device_drop_defers_cleanup_until_the_receiver_lease_is_released() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        let direct = HydraSdr::from_control(control);
        let shared = Arc::new(Mutex::new(SharedDeviceState::default()));
        let receiver = ReceiverLease::acquire(&shared, direct.stream_handle())
            .expect("reserve receiver for pending start");

        assert_eq!(
            begin_device_drop(&shared),
            DeviceDropAction::DeferredToReceiver
        );
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 0);

        direct
            .receiver_mode(ReceiverMode::Rx)
            .wait()
            .expect("complete pending receiver start");
        let cleanup = receiver
            .release()
            .expect("receiver lease owns deferred device cleanup");
        cleanup.cleanup().wait().expect("deferred device cleanup");

        assert_eq!(
            *state
                .control_out_requests
                .lock()
                .expect("control request lock"),
            [
                VendorControlRequest::receiver_mode(ReceiverMode::Rx),
                VendorControlRequest::receiver_mode(ReceiverMode::Off),
                VendorControlRequest::set_rf_bias(0),
            ]
        );
        let shared = lock_shared(&shared);
        assert_eq!(shared.device, DeviceLifecycle::Closed);
        assert_eq!(shared.receiver, ReceiverSlot::Idle);
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn dropped_receiver_lease_performs_deferred_device_cleanup() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        let direct = HydraSdr::from_control(control);
        let shared = Arc::new(Mutex::new(SharedDeviceState::default()));
        let receiver = ReceiverLease::acquire(&shared, direct.stream_handle())
            .expect("reserve receiver for pending start");

        assert_eq!(
            begin_device_drop(&shared),
            DeviceDropAction::DeferredToReceiver
        );
        direct
            .receiver_mode(ReceiverMode::Rx)
            .wait()
            .expect("complete pending receiver start");
        drop(receiver);

        assert_eq!(
            *state
                .control_out_requests
                .lock()
                .expect("control request lock"),
            [
                VendorControlRequest::receiver_mode(ReceiverMode::Rx),
                VendorControlRequest::receiver_mode(ReceiverMode::Off),
                VendorControlRequest::set_rf_bias(0),
            ]
        );
        let shared = lock_shared(&shared);
        assert_eq!(shared.device, DeviceLifecycle::Closed);
        assert_eq!(shared.receiver, ReceiverSlot::Idle);
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn failed_deferred_cleanup_is_retried_by_the_guard() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        let direct = HydraSdr::from_control(control);
        let shared = Arc::new(Mutex::new(SharedDeviceState::default()));
        let receiver = ReceiverLease::acquire(&shared, direct.stream_handle())
            .expect("reserve receiver for pending start");

        assert_eq!(
            begin_device_drop(&shared),
            DeviceDropAction::DeferredToReceiver
        );
        direct
            .receiver_mode(ReceiverMode::Rx)
            .wait()
            .expect("complete pending receiver start");
        state.fail_control_out_at.store(2, Ordering::SeqCst);
        let cleanup = receiver
            .release()
            .expect("receiver lease owns deferred device cleanup");

        assert!(cleanup.cleanup().wait().is_err());

        assert_eq!(
            *state
                .control_out_requests
                .lock()
                .expect("control request lock"),
            [
                VendorControlRequest::receiver_mode(ReceiverMode::Rx),
                VendorControlRequest::receiver_mode(ReceiverMode::Off),
                VendorControlRequest::set_rf_bias(0),
                VendorControlRequest::receiver_mode(ReceiverMode::Off),
                VendorControlRequest::set_rf_bias(0),
            ]
        );
        let shared = lock_shared(&shared);
        assert_eq!(shared.device, DeviceLifecycle::Closed);
        assert_eq!(shared.receiver, ReceiverSlot::Idle);
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn explicit_shutdown_turns_off_receiver_and_bias_power() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        let mut device = fake_device(control);

        device.shutdown().wait().expect("explicit shutdown");

        assert_eq!(
            *state
                .control_out_requests
                .lock()
                .expect("control request lock"),
            [
                VendorControlRequest::receiver_mode(ReceiverMode::Off),
                VendorControlRequest::set_rf_bias(0),
            ]
        );
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn shutdown_attempts_bias_off_after_receiver_off_fails() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        state.fail_control_out_at.store(1, Ordering::SeqCst);
        let mut device = fake_device(control);

        let error = device
            .shutdown()
            .wait()
            .expect_err("receiver shutdown should fail");

        assert!(matches!(
            error,
            Error::Operation {
                operation: "turning off the receiver during shutdown",
                ..
            }
        ));
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);
        assert_eq!(
            *state
                .control_out_requests
                .lock()
                .expect("control request lock"),
            [
                VendorControlRequest::receiver_mode(ReceiverMode::Off),
                VendorControlRequest::set_rf_bias(0),
            ]
        );

        drop(device);
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 4);
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn initial_packing_failure_does_not_enable_bias_power() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        state.fail_packing.store(true, Ordering::SeqCst);
        let config = Config::builder()
            .bias_tee(true)
            .build()
            .expect("valid fake configuration");

        HydraSdr::from_control(control)
            .into_configured(config)
            .wait()
            .expect_err("packing should fail before enabling bias power");

        assert_eq!(
            *state
                .control_out_requests
                .lock()
                .expect("control request lock"),
            [VendorControlRequest::set_frequency(100_000_000)]
        );
    }

    #[test]
    fn failed_initial_configuration_runs_full_hardware_cleanup() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            state.fail_packing.store(true, Ordering::SeqCst);
            let config = Config::builder()
                .bias_tee(true)
                .build()
                .expect("valid fake configuration");
            let direct = HydraSdr::from_control(control);
            let cleanup = OpenCleanupGuard::new(&direct);
            let setup = direct.into_configured(config);

            let result = setup
                .continue_with(move |result| match result {
                    Ok(_) => {
                        cleanup.disarm();
                        Either::left(ready(Ok(())))
                    }
                    Err(error) => Either::right(cleanup.cleanup().map(move |_| Err(error))),
                })
                .await;

            assert!(result.is_err());
            assert_eq!(
                *state
                    .control_out_requests
                    .lock()
                    .expect("control request lock"),
                [
                    VendorControlRequest::set_frequency(100_000_000),
                    VendorControlRequest::receiver_mode(ReceiverMode::Off),
                    VendorControlRequest::set_rf_bias(0),
                ]
            );
        });
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn dropping_open_transaction_guard_runs_hardware_cleanup() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        let direct = HydraSdr::from_control(control);

        drop(OpenCleanupGuard::new(&direct));

        assert_eq!(
            *state
                .control_out_requests
                .lock()
                .expect("control request lock"),
            [
                VendorControlRequest::receiver_mode(ReceiverMode::Off),
                VendorControlRequest::set_rf_bias(0),
            ]
        );
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn cancelling_initial_configuration_runs_hardware_cleanup() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        state.pause_control_out_at.store(1, Ordering::SeqCst);
        let config = Config::default();
        let direct = HydraSdr::from_control(control);
        let cleanup = OpenCleanupGuard::new(&direct);
        let setup = direct.into_configured(config);
        let transaction = setup.continue_with(move |result| match result {
            Ok(_) => {
                cleanup.disarm();
                Either::left(ready(Ok(())))
            }
            Err(error) => Either::right(cleanup.cleanup().map(move |_| Err(error))),
        });
        let mut transaction = Box::pin(transaction.into_future());

        assert!(block_on(futures_lite::future::poll_once(transaction.as_mut())).is_none());
        drop(transaction);

        assert_eq!(
            *state
                .control_out_requests
                .lock()
                .expect("control request lock"),
            [
                VendorControlRequest::set_frequency(100_000_000),
                VendorControlRequest::receiver_mode(ReceiverMode::Off),
                VendorControlRequest::set_rf_bias(0),
            ]
        );
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn targeted_bias_tee_control_sends_one_request() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        let mut device = fake_device(control);

        device.set_bias_tee(true).wait().expect("enable bias tee");

        assert_eq!(
            *state
                .control_out_requests
                .lock()
                .expect("control request lock"),
            [VendorControlRequest::set_rf_bias(1)]
        );
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn targeted_packing_control_updates_streaming_state() {
        let mut device = fake_device(FakeControl::default());

        device.set_packing(true).wait().expect("enable packing");

        assert!(device.direct.streaming_packing_enabled());
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn targeted_decimation_policy_reapplies_the_requested_rate() {
        let mut device = fake_device(FakeControl::default());

        device
            .set_decimation_policy(5_000_000, crate::DecimationPolicy::HighDefinition)
            .wait()
            .expect("select high-definition decimation");

        assert_eq!(device.direct.streaming_decimation_factor(), 2);
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn dropping_unpolled_shutdown_keeps_native_fallback_armed() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        let mut device = fake_device(control);

        let shutdown = device.shutdown();
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 0);
        drop(shutdown);
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 0);
        drop(device);

        assert_eq!(
            *state
                .control_out_requests
                .lock()
                .expect("control request lock"),
            [
                VendorControlRequest::receiver_mode(ReceiverMode::Off),
                VendorControlRequest::set_rf_bias(0),
            ]
        );
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn canceling_polled_shutdown_keeps_native_fallback_armed() {
        use std::future::IntoFuture;

        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        state.pause_control_out_at.store(1, Ordering::SeqCst);
        let mut device = fake_device(control);
        let mut shutdown = Box::pin(device.shutdown().into_future());

        assert!(block_on(futures_lite::future::poll_once(shutdown.as_mut())).is_none());
        drop(shutdown);
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 1);
        drop(device);

        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 3);
        assert_eq!(
            *state
                .control_out_requests
                .lock()
                .expect("control request lock"),
            [
                VendorControlRequest::receiver_mode(ReceiverMode::Off),
                VendorControlRequest::receiver_mode(ReceiverMode::Off),
                VendorControlRequest::set_rf_bias(0),
            ]
        );
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn native_device_drop_performs_best_effort_shutdown() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        let device = fake_device(control);

        drop(device);

        assert_eq!(
            *state
                .control_out_requests
                .lock()
                .expect("control request lock"),
            [
                VendorControlRequest::receiver_mode(ReceiverMode::Off),
                VendorControlRequest::set_rf_bias(0),
            ]
        );
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn owned_raw_stream_drop_retries_failed_receiver_off() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        let device = fake_device(control);

        {
            let mut stream = OwnedRawRxStreamInner::new(device);
            stream.start().expect("start owned raw stream");
            state.fail_control_out_at.store(3, Ordering::SeqCst);
            stream
                .stop()
                .expect_err("first receiver-off request should fail");
        }

        assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
        assert_eq!(
            *state
                .control_out_requests
                .lock()
                .expect("control request lock"),
            [
                VendorControlRequest::receiver_mode(ReceiverMode::Off),
                VendorControlRequest::receiver_mode(ReceiverMode::Rx),
                VendorControlRequest::receiver_mode(ReceiverMode::Off),
                VendorControlRequest::receiver_mode(ReceiverMode::Off),
                VendorControlRequest::receiver_mode(ReceiverMode::Off),
                VendorControlRequest::set_rf_bias(0),
            ]
        );
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn owned_synchronous_f32_stream_keeps_queue_between_reads() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        let device = fake_device(control);
        let mut stream = F32RxStreamInner::new(device);
        stream.start().expect("start owned synchronous F32 stream");

        let mut first = [Complex32::default(); 1];
        let mut second = [Complex32::default(); 1];
        assert_eq!(
            stream
                .read(&mut first, Duration::from_secs(1))
                .expect("first read"),
            1
        );
        assert_eq!(
            stream
                .read(&mut second, Duration::from_secs(1))
                .expect("second read"),
            1
        );
        assert_eq!(state.bulk_in_count.load(Ordering::SeqCst), 1);
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);

        let stats = stream.stop().expect("stop owned synchronous F32 stream");
        assert_eq!(stats.buffers_received, 1);
        assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 3);
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn synchronous_read_error_requires_stop_before_restart() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        let device = fake_device(control);
        let mut stream = F32RxStreamInner::new(device);
        let mut sample = [Complex32::default(); 1];

        stream.start().expect("start synchronous stream");
        state.fail_bulk_completion.store(true, Ordering::SeqCst);
        assert!(stream.read(&mut sample, Duration::MAX).is_err());
        assert_eq!(stream.state, ReceiverState::CleanupRequired);
        assert!(
            stream
                .start()
                .is_err_and(|error| error.kind() == crate::ErrorKind::Busy)
        );

        stream.stop().expect("clean up failed read");
        stream.start().expect("restart after cleanup");
        stream.stop().expect("stop restarted stream");
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn synchronous_raw_read_error_requires_stop_before_restart() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        let device = fake_device(control);
        let mut stream = OwnedRawRxStreamInner::new(device);

        stream.start().expect("start synchronous raw stream");
        state.fail_bulk_completion.store(true, Ordering::SeqCst);
        assert!(stream.next_block(Duration::MAX).is_err());
        assert_eq!(stream.state, ReceiverState::CleanupRequired);
        assert!(
            stream
                .start()
                .is_err_and(|error| error.kind() == crate::ErrorKind::Busy)
        );

        stream.stop().expect("clean up failed raw block");
        stream.start().expect("restart raw stream after cleanup");
        stream.stop().expect("stop restarted raw stream");
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn owned_synchronous_stream_stats_accumulate_across_restarts() {
        let control = FakeControl::default();
        let device = fake_device(control);
        let mut stream = F32RxStreamInner::new(device);
        let mut sample = [Complex32::default(); 1];

        stream.start().expect("start first synchronous run");
        stream
            .read(&mut sample, Duration::from_secs(1))
            .expect("read first synchronous run");
        assert_eq!(
            stream
                .stop()
                .expect("stop first synchronous run")
                .buffers_received,
            1
        );

        stream.start().expect("start second synchronous run");
        stream
            .read(&mut sample, Duration::from_secs(1))
            .expect("read second synchronous run");
        assert_eq!(
            stream
                .stop()
                .expect("stop second synchronous run")
                .buffers_received,
            2
        );
    }

    #[test]
    fn owned_async_f32_stream_reuses_queue_and_closes_on_drop() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control);
            let mut stream = AsyncF32RxStreamInner::new(device);
            stream.start().await.expect("start owned async F32 stream");

            let mut first = [Complex32::default(); 1];
            let mut second = [Complex32::default(); 1];
            assert_eq!(stream.read(&mut first).await.expect("first read"), 1);
            assert_eq!(stream.read(&mut second).await.expect("second read"), 1);
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);

            let stats = stream.stop().await.expect("stop owned async F32 stream");
            drop(stream);
            assert_eq!(stats.buffers_received, 1);
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 5);
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 2);
        });
    }

    #[test]
    fn owned_async_f32_stream_reuses_queue_across_restart() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control);
            let mut stream = AsyncF32RxStreamInner::new(device);

            stream.start().await.expect("start owned async F32 stream");
            let mut first = [Complex32::default(); 1];
            assert_eq!(stream.read(&mut first).await.expect("first read"), 1);
            stream.stop().await.expect("stop owned async F32 stream");
            assert_eq!(state.bulk_in_count.load(Ordering::SeqCst), 1);
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
            assert!(
                stream
                    .read(&mut first)
                    .await
                    .is_err_and(|error| { error.kind() == crate::ErrorKind::StreamClosed })
            );
            stream.stop().await.expect("repeated stop is a no-op");
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 3);

            stream
                .start()
                .await
                .expect("restart owned async F32 stream");
            let mut second = [Complex32::default(); 1];
            assert_eq!(stream.read(&mut second).await.expect("second read"), 1);
            let stats = stream.stop().await.expect("stop owned async F32 stream");
            assert_eq!(state.bulk_in_count.load(Ordering::SeqCst), 1);
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 2);
            assert_eq!(
                stats.buffers_discarded_on_restart,
                crate::rfone::RFONE_TRANSFER_COUNT as u64
            );

            drop(stream);
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 3);
        });
    }

    #[test]
    fn owned_async_stream_stats_survive_queue_recreation() {
        block_on(async {
            let control = FakeControl::default();
            let device = fake_device(control);
            let mut stream = AsyncF32RxStreamInner::new(device);
            let mut sample = [Complex32::default(); 1];

            stream.start().await.expect("start first async queue");
            stream
                .read(&mut sample)
                .await
                .expect("read first async queue");
            stream.stop().await.expect("stop first async queue");
            stream.retire_stream();

            stream.start().await.expect("start replacement async queue");
            stream
                .read(&mut sample)
                .await
                .expect("read replacement async queue");
            assert_eq!(
                stream
                    .stop()
                    .await
                    .expect("stop replacement async queue")
                    .buffers_received,
                2
            );
        });
    }

    #[test]
    fn async_raw_read_error_requires_stop_before_restart() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control);
            let mut stream = AsyncRawRxStreamInner::new(device);

            stream.start().await.expect("start async raw stream");
            state.fail_bulk_completion.store(true, Ordering::SeqCst);
            assert!(stream.next_block().await.is_err());
            assert_eq!(stream.state, ReceiverState::CleanupRequired);
            assert!(
                stream
                    .start()
                    .await
                    .is_err_and(|error| error.kind() == crate::ErrorKind::Busy)
            );

            stream.stop().await.expect("clean up failed read");
            stream.start().await.expect("restart after cleanup");
            stream.stop().await.expect("stop restarted stream");
        });
    }

    #[test]
    fn async_f32_read_error_requires_stop_before_restart() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control);
            let mut stream = AsyncF32RxStreamInner::new(device);
            let mut sample = [Complex32::default(); 1];

            stream.start().await.expect("start async F32 stream");
            state.fail_bulk_completion.store(true, Ordering::SeqCst);
            assert!(stream.read(&mut sample).await.is_err());
            assert_eq!(stream.state, ReceiverState::CleanupRequired);
            assert!(
                stream
                    .start()
                    .await
                    .is_err_and(|error| error.kind() == crate::ErrorKind::Busy)
            );

            stream.stop().await.expect("clean up failed F32 read");
            stream.start().await.expect("restart F32 after cleanup");
            stream.stop().await.expect("stop restarted F32 stream");
        });
    }

    #[test]
    fn cancelled_owned_async_raw_start_can_be_stopped() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            state.pause_control_out_at.store(2, Ordering::SeqCst);
            let device = fake_device(control);
            let mut stream = AsyncRawRxStreamInner::new(device);

            let mut start = Box::pin(stream.start());
            assert!(futures_lite::future::poll_once(&mut start).await.is_none());
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);
            drop(start);

            stream
                .stop()
                .await
                .expect("stop cancelled owned async raw stream start");
            drop(stream);
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 5);
        });
    }

    #[test]
    fn cancelled_owned_async_f32_start_can_be_stopped() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            state.pause_control_out_at.store(2, Ordering::SeqCst);
            let device = fake_device(control);
            let mut stream = AsyncF32RxStreamInner::new(device);

            let mut start = Box::pin(stream.start());
            assert!(futures_lite::future::poll_once(&mut start).await.is_none());
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);
            drop(start);

            stream
                .stop()
                .await
                .expect("stop cancelled owned async F32 stream start");
            drop(stream);
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 5);
        });
    }

    #[test]
    fn owned_async_stream_stop_error_still_closes_on_drop() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control);
            let mut stream = AsyncF32RxStreamInner::new(device);
            stream.start().await.expect("start owned async F32 stream");
            state.fail_control_out.store(true, Ordering::SeqCst);

            assert!(
                stream
                    .stop()
                    .await
                    .is_err_and(|error| error.kind() == crate::ErrorKind::Usb)
            );
            drop(stream);
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
        });
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn native_drop_stops_async_receiver_closes_queue_and_shuts_down_hardware() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control);
            let mut stream = AsyncF32RxStreamInner::new(device);
            stream.start().await.expect("start owned async F32 stream");
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);

            drop(stream);
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 5);
        });
    }
}
