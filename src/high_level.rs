//! High-level synchronous and asynchronous HydraSDR RFOne driver interface.

use core::marker::PhantomData;
use nusb::MaybeFuture;
use std::future::{Future, IntoFuture};
use std::pin::Pin;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::task::{Context, Poll};

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

#[derive(Debug)]
pub(crate) struct DeviceInner<C: ControlBackend + 'static = NusbControl> {
    direct: HydraSdr<C>,
    info: DeviceInfo,
    shared: Arc<DeviceShared<C>>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum DeviceLifecycle {
    Open,
    Closing,
    Closed,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ReceiverState {
    Stopped,
    CleanupRequired,
    Running,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ClaimAvailability {
    Available,
    Held,
    CleanupRequired,
}

#[derive(Debug)]
struct SharedDeviceState {
    device: DeviceLifecycle,
    claim: ClaimAvailability,
}

impl Default for SharedDeviceState {
    fn default() -> Self {
        Self {
            device: DeviceLifecycle::Open,
            claim: ClaimAvailability::Available,
        }
    }
}

/// The final owner performs hardware shutdown. Control configuration stays on
/// DeviceInner; this handle shares its backend and live streaming parameters.
#[derive(Debug)]
struct DeviceShared<C: ControlBackend + 'static> {
    direct: HydraSdr<C>,
    state: Mutex<SharedDeviceState>,
}

impl<C: ControlBackend + 'static> DeviceShared<C> {
    fn lock(&self) -> MutexGuard<'_, SharedDeviceState> {
        self.state.lock().unwrap_or_else(PoisonError::into_inner)
    }

    fn ensure_open(&self) -> Result<()> {
        if self.lock().device == DeviceLifecycle::Open {
            Ok(())
        } else {
            Err(Error::DeviceClosed)
        }
    }

    fn begin_shutdown(&self) -> Result<bool> {
        let mut state = self.lock();
        match state.device {
            DeviceLifecycle::Closed => Ok(false),
            DeviceLifecycle::Closing => Ok(true),
            DeviceLifecycle::Open => {
                if state.claim == ClaimAvailability::Held {
                    return Err(Error::Busy);
                }
                state.device = DeviceLifecycle::Closing;
                Ok(true)
            }
        }
    }

    fn shutdown(self: &Arc<Self>) -> impl MaybeFuture<Output = Result<()>> + use<C> {
        let shared = Arc::clone(self);
        ready(()).map(move |()| shared.begin_shutdown()).and_then({
            let shared = Arc::clone(self);
            move |run| {
                if run {
                    Either::left(shutdown_hardware(&shared.direct).map(move |result| {
                        if result.is_ok() {
                            let mut state = shared.lock();
                            state.device = DeviceLifecycle::Closed;
                            state.claim = ClaimAvailability::Available;
                        }
                        result
                    }))
                } else {
                    Either::right(ready(Ok(())))
                }
            }
        })
    }
}

impl<C: ControlBackend + 'static> Drop for DeviceShared<C> {
    fn drop(&mut self) {
        if self.lock().device == DeviceLifecycle::Closed {
            return;
        }
        let cleanup = shutdown_hardware(&self.direct);
        #[cfg(not(target_arch = "wasm32"))]
        let _ = cleanup.wait();
        #[cfg(target_arch = "wasm32")]
        wasm_bindgen_futures::spawn_local(async move {
            let _ = cleanup.await;
        });
    }
}

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
///     let stats = rx.close().wait()?;
///     println!("{stats:?}");
///     dev.shutdown().wait()?;
///
///     Ok(())
/// }
/// ```
#[derive(Debug)]
pub struct Device<M: SampleMode = F32Iq> {
    inner: DeviceInner<NusbControl>,
    config: Config<M>,
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
        let lifecycle = self.inner.shared.ensure_open();
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
        let lifecycle = self.inner.shared.ensure_open();
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
        let lifecycle = self.inner.shared.ensure_open();
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
        let lifecycle = self.inner.shared.ensure_open();
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
        let lifecycle = self.inner.shared.ensure_open();
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
        let lifecycle = self.inner.shared.ensure_open();
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
        self.inner.shutdown()
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
        Ok(RxStream::new(RxStreamClaim::acquire(&self.inner.shared)?))
    }
}

impl Device<RawAdc> {
    /// Enable or disable packed raw-sample transfers.
    ///
    /// An active raw stream adopts the new transfer format on its next block.
    pub fn set_packing(&mut self, enabled: bool) -> impl MaybeFuture<Output = Result<()>> {
        let lifecycle = self.inner.shared.ensure_open();
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
        let lifecycle = self.inner.shared.ensure_open();
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

impl<C: ControlBackend + 'static> DeviceInner<C> {
    fn new(direct: HydraSdr<C>, info: DeviceInfo) -> Self {
        let shared = Arc::new(DeviceShared {
            direct: direct.stream_handle(),
            state: Mutex::new(SharedDeviceState::default()),
        });
        Self {
            direct,
            info,
            shared,
        }
    }

    pub(crate) fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn shutdown(&mut self) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        self.shared.shutdown()
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
                            inner: DeviceInner::new(direct, info),
                            config: saved_config,
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

/// Owned synchronous raw ADC stream state used by the typed stream facade.
#[cfg(not(target_arch = "wasm32"))]
struct OwnedRawRxStreamInner<C: ControlBackend + StreamingBackend + 'static> {
    stream: Option<DirectRawRxStream<C::BulkIn>>,
    stats: StreamingStats,
    // Fields drop in declaration order: release the queue before its claim.
    claim: RxStreamClaim<C>,
}

#[cfg(not(target_arch = "wasm32"))]
impl<C> OwnedRawRxStreamInner<C>
where
    C: ControlBackend + StreamingBackend + 'static,
{
    fn new(claim: RxStreamClaim<C>) -> Self {
        Self {
            stream: None,
            stats: StreamingStats::default(),
            claim,
        }
    }

    fn start(&mut self) -> Result<()> {
        if !self.claim.begin_start()? {
            return Ok(());
        }
        let device = &self.claim.shared;
        self.stream = Some(device.direct.start_raw_rx_stream()?);
        self.claim.set_receiver_state(ReceiverState::Running);
        Ok(())
    }

    fn next_block(&mut self, timeout: Duration) -> Result<Option<SampleBlock<'_>>> {
        if self.claim.receiver_state() != ReceiverState::Running {
            return Err(Error::stream_closed("raw RX stream is stopped"));
        }
        let desired_packing = self.claim.shared.direct.streaming_packing_enabled();
        let packing_changed = self
            .stream
            .as_ref()
            .is_some_and(|stream| stream.packing_enabled() != desired_packing);
        if packing_changed {
            self.claim
                .set_receiver_state(ReceiverState::CleanupRequired);
            let device = &self.claim.shared;
            let stream = self.stream.take().expect("running raw stream exists");
            let (stats, result) = device.direct.close_raw_rx_stream(stream);
            self.stats.accumulate(stats);
            result?;
            self.stream = Some(device.direct.start_raw_rx_stream()?);
            self.claim.set_receiver_state(ReceiverState::Running);
        }
        let stream = self
            .stream
            .as_mut()
            .ok_or(Error::stream_closed("raw RX stream is closed"))?;
        match stream.next_transfer(timeout) {
            Ok(transfer) => Ok(transfer.map(SampleBlock::from_transfer)),
            Err(error) => {
                self.claim
                    .set_receiver_state(ReceiverState::CleanupRequired);
                Err(error)
            }
        }
    }

    fn stop(&mut self) -> Result<StreamingStats> {
        if !self.claim.begin_stop() {
            return Ok(self.stats);
        }
        let device = &self.claim.shared;
        let result = if let Some(stream) = self.stream.take() {
            let (stats, result) = device.direct.close_raw_rx_stream(stream);
            self.stats.accumulate(stats);
            result
        } else {
            device.direct.receiver_mode(ReceiverMode::Off).wait()
        };
        result?;
        self.claim.set_receiver_state(ReceiverState::Stopped);
        Ok(self.stats)
    }
}

/// Owned synchronous converted `F32Iq` stream state.
#[cfg(not(target_arch = "wasm32"))]
struct F32RxStreamInner<C: ControlBackend + StreamingBackend + 'static> {
    stream: Option<DirectRxStream<C::BulkIn>>,
    stats: StreamingStats,
    // Fields drop in declaration order: release the queue before its claim.
    claim: RxStreamClaim<C>,
}

#[cfg(not(target_arch = "wasm32"))]
impl<C> F32RxStreamInner<C>
where
    C: ControlBackend + StreamingBackend + 'static,
{
    fn new(claim: RxStreamClaim<C>) -> Self {
        Self {
            stream: None,
            stats: StreamingStats::default(),
            claim,
        }
    }

    fn start(&mut self) -> Result<()> {
        if !self.claim.begin_start()? {
            return Ok(());
        }
        let device = &self.claim.shared;
        self.stream = Some(device.direct.start_rx_stream()?);
        self.claim.set_receiver_state(ReceiverState::Running);
        Ok(())
    }

    /// Read converted complex samples into `out`.
    fn read(&mut self, out: &mut [Complex32], timeout: Duration) -> Result<usize> {
        if self.claim.receiver_state() != ReceiverState::Running {
            return Err(Error::stream_closed("synchronous F32 RX stream is stopped"));
        }
        let factor = self.claim.shared.direct.streaming_decimation_factor();
        let stream = self
            .stream
            .as_mut()
            .ok_or(Error::stream_closed("F32 RX stream is closed"))?;
        stream.set_decimation_factor(factor)?;
        let result = stream.read_float32_iq(out, timeout);
        if result.is_err() {
            self.claim
                .set_receiver_state(ReceiverState::CleanupRequired);
        }
        result
    }

    /// Request receiver-off cleanup. Repeated calls are no-ops.
    fn stop(&mut self) -> Result<StreamingStats> {
        if !self.claim.begin_stop() {
            return Ok(self.stats);
        }
        let device = &self.claim.shared;
        let result = if let Some(stream) = self.stream.take() {
            let (stats, result) = device.direct.close_rx_stream(stream);
            self.stats.accumulate(stats);
            result
        } else {
            device.direct.receiver_mode(ReceiverMode::Off).wait()
        };
        result?;
        self.claim.set_receiver_state(ReceiverState::Stopped);
        Ok(self.stats)
    }
}

/// Owned async stream state for raw ADC blocks.
struct AsyncRawRxStreamInner<C: ControlBackend + AsyncStreamingBackend + 'static> {
    stream: Option<DirectAsyncRawRxStream<C::BulkIn>>,
    stats: StreamingStats,
    // Fields drop in declaration order: release the queue before its claim.
    claim: RxStreamClaim<C>,
}

impl<C> AsyncRawRxStreamInner<C>
where
    C: ControlBackend + AsyncStreamingBackend + 'static,
{
    fn new(claim: RxStreamClaim<C>) -> Self {
        Self {
            stream: None,
            stats: StreamingStats::default(),
            claim,
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

    fn packing_changed(&self) -> bool {
        let desired_packing = self.claim.shared.direct.streaming_packing_enabled();
        self.stream
            .as_ref()
            .is_some_and(|stream| stream.packing_enabled() != desired_packing)
    }

    async fn start(&mut self) -> Result<()> {
        if !self.claim.begin_start()? {
            return Ok(());
        }
        let desired_packing = self.claim.shared.direct.streaming_packing_enabled();
        let stream_reusable = self.stream.as_ref().is_some_and(|stream| {
            !stream.is_closed() && stream.packing_enabled() == desired_packing
        });
        if !stream_reusable {
            self.retire_stream();
        }
        let device = &self.claim.shared;
        if stream_reusable {
            device.direct.receiver_mode(ReceiverMode::Rx).await?;
            self.claim.set_receiver_state(ReceiverState::Running);
            return Ok(());
        }
        let stream = device.direct.start_raw_rx_stream_async().await?;
        self.stream = Some(stream);
        self.claim.set_receiver_state(ReceiverState::Running);
        Ok(())
    }

    /// Read the next sample block.
    async fn next_block(&mut self) -> Result<Option<SampleBlock<'_>>> {
        if self.claim.receiver_state() != ReceiverState::Running {
            return Err(Error::stream_closed("async raw RX stream is stopped"));
        }
        if self.packing_changed() {
            self.retire_stream();
            self.claim
                .set_receiver_state(ReceiverState::CleanupRequired);
            let device = &self.claim.shared;
            self.stream = Some(device.direct.start_raw_rx_stream_async().await?);
            self.claim.set_receiver_state(ReceiverState::Running);
        }
        let stream = self
            .stream
            .as_mut()
            .ok_or(Error::stream_closed("async RX stream is closed"))?;
        match stream.next_transfer().await {
            Ok(transfer) => Ok(transfer.map(SampleBlock::from_transfer)),
            Err(error) => {
                self.claim
                    .set_receiver_state(ReceiverState::CleanupRequired);
                Err(error)
            }
        }
    }

    fn poll_next_block(&mut self, cx: &mut Context<'_>) -> Poll<Result<()>> {
        if self.claim.receiver_state() != ReceiverState::Running {
            return Poll::Ready(Err(Error::stream_closed("async raw RX stream is stopped")));
        }
        let result = match self.stream.as_mut() {
            Some(stream) => stream.poll_next_transfer(cx),
            None => Poll::Ready(Err(Error::stream_closed("async RX stream is closed"))),
        };
        if matches!(result, Poll::Ready(Err(_))) {
            self.claim
                .set_receiver_state(ReceiverState::CleanupRequired);
        }
        result
    }

    fn current_block(&self) -> Result<SampleBlock<'_>> {
        self.stream
            .as_ref()
            .ok_or(Error::stream_closed("async RX stream is closed"))?
            .current_transfer()
            .map(SampleBlock::from_transfer)
    }

    async fn stop(&mut self) -> Result<StreamingStats> {
        if !self.claim.begin_stop() {
            return Ok(self.current_stats());
        }
        self.claim
            .shared
            .direct
            .receiver_mode(ReceiverMode::Off)
            .await?;
        if let Some(stream) = self.stream.as_mut() {
            stream.pause()?;
        }
        self.claim.set_receiver_state(ReceiverState::Stopped);
        Ok(self.current_stats())
    }
}

/// Owned async stream state for converted `F32Iq` samples.
struct AsyncF32RxStreamInner<C: ControlBackend + AsyncStreamingBackend + 'static> {
    stream: Option<AsyncDirectRxStream<C::BulkIn>>,
    stats: StreamingStats,
    // Fields drop in declaration order: release the queue before its claim.
    claim: RxStreamClaim<C>,
}

impl<C> AsyncF32RxStreamInner<C>
where
    C: ControlBackend + AsyncStreamingBackend + 'static,
{
    fn new(claim: RxStreamClaim<C>) -> Self {
        Self {
            stream: None,
            stats: StreamingStats::default(),
            claim,
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
        if !self.claim.begin_start()? {
            return Ok(());
        }
        let device = &self.claim.shared;
        if let Some(stream) = self.stream.as_mut() {
            stream.set_decimation_factor(device.direct.streaming_decimation_factor())?;
            device.direct.receiver_mode(ReceiverMode::Rx).await?;
            self.claim.set_receiver_state(ReceiverState::Running);
            return Ok(());
        }
        let stream = device.direct.start_rx_stream_async().await?;
        self.stream = Some(stream);
        self.claim.set_receiver_state(ReceiverState::Running);
        Ok(())
    }

    fn begin_read(&mut self) -> Result<()> {
        if self.claim.receiver_state() != ReceiverState::Running {
            return Err(Error::stream_closed("async F32 RX stream is stopped"));
        }
        let factor = self.claim.shared.direct.streaming_decimation_factor();
        let stream = self
            .stream
            .as_mut()
            .ok_or(Error::stream_closed("async F32 RX stream is closed"))?;
        stream.set_decimation_factor(factor)
    }

    fn poll_read(&mut self, out: &mut [Complex32], cx: &mut Context<'_>) -> Poll<Result<usize>> {
        let result = match self.stream.as_mut() {
            Some(stream) => stream.poll_read_float32_iq(out, cx),
            None => Poll::Ready(Err(Error::stream_closed("async F32 RX stream is closed"))),
        };
        match result {
            Poll::Ready(Err(error)) => {
                self.retire_stream();
                self.claim
                    .set_receiver_state(ReceiverState::CleanupRequired);
                Poll::Ready(Err(error))
            }
            result => result,
        }
    }

    /// Read converted complex samples into `out`.
    #[cfg(test)]
    async fn read(&mut self, out: &mut [Complex32]) -> Result<usize> {
        self.begin_read()?;
        std::future::poll_fn(|cx| self.poll_read(out, cx)).await
    }

    async fn stop(&mut self) -> Result<StreamingStats> {
        if !self.claim.begin_stop() {
            return Ok(self.current_stats());
        }
        self.claim
            .shared
            .direct
            .receiver_mode(ReceiverMode::Off)
            .await?;
        if let Some(stream) = self.stream.as_mut() {
            stream.pause()?;
        }
        self.claim.set_receiver_state(ReceiverState::Stopped);
        Ok(self.current_stats())
    }
}

#[cfg(not(target_arch = "wasm32"))]
trait RxBackend: ControlBackend + StreamingBackend + AsyncStreamingBackend + 'static {}
#[cfg(not(target_arch = "wasm32"))]
impl<C: ControlBackend + StreamingBackend + AsyncStreamingBackend + 'static> RxBackend for C {}
#[cfg(target_arch = "wasm32")]
trait RxBackend: ControlBackend + AsyncStreamingBackend + 'static {}
#[cfg(target_arch = "wasm32")]
impl<C: ControlBackend + AsyncStreamingBackend + 'static> RxBackend for C {}

enum RxStreamState<C: RxBackend = NusbControl> {
    Dormant(RxStreamClaim<C>),
    Poisoned,
    #[cfg(not(target_arch = "wasm32"))]
    BlockingRaw(OwnedRawRxStreamInner<C>),
    #[cfg(not(target_arch = "wasm32"))]
    BlockingF32(Box<F32RxStreamInner<C>>),
    AsyncRaw(AsyncRawRxStreamInner<C>),
    AsyncF32(Box<AsyncF32RxStreamInner<C>>),
}

impl<C: RxBackend> RxStreamState<C> {
    fn into_close_parts(self) -> Result<(RxStreamClaim<C>, StreamingStats)> {
        let parts = match self {
            RxStreamState::Dormant(claim) => (claim, StreamingStats::default()),
            RxStreamState::AsyncRaw(mut inner) => {
                inner.retire_stream();
                (inner.claim, inner.stats)
            }
            RxStreamState::AsyncF32(mut inner) => {
                inner.retire_stream();
                (inner.claim, inner.stats)
            }
            #[cfg(not(target_arch = "wasm32"))]
            RxStreamState::BlockingRaw(mut inner) => {
                if let Some(mut stream) = inner.stream.take() {
                    inner.stats.accumulate(stream.close());
                }
                (inner.claim, inner.stats)
            }
            #[cfg(not(target_arch = "wasm32"))]
            RxStreamState::BlockingF32(mut inner) => {
                if let Some(mut stream) = inner.stream.take() {
                    inner.stats.accumulate(stream.close());
                }
                (inner.claim, inner.stats)
            }
            RxStreamState::Poisoned => return Err(Error::stream_closed("RX stream has no device")),
        };
        Ok(parts)
    }
}

/// Keeps both the device and its exclusive stream claim alive through cleanup.
#[derive(Debug)]
struct RxStreamClaim<C: ControlBackend + 'static> {
    shared: Arc<DeviceShared<C>>,
    receiver: ReceiverState,
    cleanup_armed: bool,
}

impl<C: ControlBackend + 'static> RxStreamClaim<C> {
    fn receiver_state(&self) -> ReceiverState {
        self.receiver
    }

    fn set_receiver_state(&mut self, receiver: ReceiverState) {
        self.receiver = receiver;
    }

    fn begin_start(&mut self) -> Result<bool> {
        match self.receiver {
            ReceiverState::Running => Ok(false),
            ReceiverState::CleanupRequired => Err(Error::Busy),
            ReceiverState::Stopped => {
                self.receiver = ReceiverState::CleanupRequired;
                Ok(true)
            }
        }
    }

    fn begin_stop(&mut self) -> bool {
        if self.receiver == ReceiverState::Stopped {
            return false;
        }
        // Cancellation or failure must not leave a stream apparently running.
        self.receiver = ReceiverState::CleanupRequired;
        true
    }

    fn acquire(shared: &Arc<DeviceShared<C>>) -> Result<Self> {
        let mut state = shared.lock();
        if state.device != DeviceLifecycle::Open {
            return Err(Error::DeviceClosed);
        }
        if state.claim != ClaimAvailability::Available {
            return Err(Error::Busy);
        }
        state.claim = ClaimAvailability::Held;
        Ok(Self {
            shared: Arc::clone(shared),
            receiver: ReceiverState::Stopped,
            cleanup_armed: true,
        })
    }

    fn release(&mut self) {
        self.shared.lock().claim = ClaimAvailability::Available;
        self.cleanup_armed = false;
    }

    fn close(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        ready(()).continue_with(move |()| {
            let operation = if self.begin_stop() {
                Either::left(self.shared.direct.receiver_mode(ReceiverMode::Off))
            } else {
                Either::right(ready(Ok(())))
            };
            operation.map(move |result| {
                if result.is_ok() {
                    self.receiver = ReceiverState::Stopped;
                    self.release();
                }
                result
            })
        })
    }

    // The owned operation keeps the claim reserved even before a WebUSB
    // background task is polled. Drop runs it to completion best-effort.
    fn cleanup(&mut self) -> impl MaybeFuture<Output = ()> + use<C> {
        self.cleanup_armed = false;
        let shared = Arc::clone(&self.shared);
        let operation = if self.begin_stop() {
            Either::left(shared.direct.receiver_mode(ReceiverMode::Off))
        } else {
            Either::right(ready(Ok(())))
        };
        operation.map(move |result| {
            let mut state = shared.lock();
            state.claim = if result.is_ok() {
                ClaimAvailability::Available
            } else {
                ClaimAvailability::CleanupRequired
            };
        })
    }
}

impl<C: ControlBackend + 'static> Drop for RxStreamClaim<C> {
    fn drop(&mut self) {
        if !self.cleanup_armed {
            return;
        }
        let cleanup = self.cleanup();
        #[cfg(not(target_arch = "wasm32"))]
        cleanup.wait();
        #[cfg(target_arch = "wasm32")]
        wasm_bindgen_futures::spawn_local(async move {
            cleanup.await;
        });
    }
}

/// Owned receive stream for sample mode `M`, defaulting to [`F32Iq`].
///
/// Create this stream with [`Device::rx_stream`]. Waiting the first
/// [`RxStream::start`] operation selects blocking USB on native targets;
/// awaiting it selects asynchronous USB. Do not mix waiting and awaiting on
/// one stream, except for [`RxStream::close`], which supports either mode.
///
/// The stream keeps its device alive and can still start, stop, and restart
/// after the public device handle is dropped. Configured RF bias power remains
/// enabled until explicit device shutdown or the final owner is dropped.
#[must_use = "RX streams retain the device's exclusive stream claim until closed or dropped"]
pub struct RxStream<M: SampleMode = F32Iq> {
    state: RxStreamState,
    mode: PhantomData<fn() -> M>,
}

impl<M: SampleMode> RxStream<M> {
    fn new(claim: RxStreamClaim<NusbControl>) -> Self {
        Self {
            state: RxStreamState::Dormant(claim),
            mode: PhantomData,
        }
    }

    /// Start reception and the persistent USB transfer queue.
    ///
    /// A stream requiring cleanup must be stopped before restarting.
    ///
    /// Call [`MaybeFuture::wait`] for blocking operation on native targets, or
    /// await the returned operation for asynchronous operation.
    pub fn start(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        StartOperation { stream: self }
    }

    /// Stop reception and return accumulated streaming counters.
    ///
    /// This preserves the stream for restart, retains its exclusive claim, and
    /// leaves RF bias power unchanged.
    /// If this operation fails or is canceled after it starts, retry `stop`
    /// before reading or restarting.
    pub fn stop(&mut self) -> impl MaybeFuture<Output = Result<StreamingStats>> + '_ {
        StopOperation { stream: self }
    }

    /// Consume the stream, stop reception, and return accumulated statistics.
    ///
    /// The returned operation owns the stream immediately, but retires its USB
    /// queue and attempts receiver-off only when polled or waited. Success
    /// releases the exclusive claim and the stream's device reference.
    ///
    /// Failure returns the original error; the stream cannot be retried. Failure
    /// or cancellation invokes best-effort drop cleanup, as does dropping an
    /// unpolled operation. WebUSB runs that fallback in the background and keeps
    /// the claim reserved until its attempt finishes.
    ///
    /// Native callers may wait or await regardless of the selected read mode.
    /// WebUSB supports awaiting only.
    ///
    /// Close moves the stream even before the operation is polled:
    ///
    /// ```compile_fail,E0382
    /// # use hydrasdr_rs::RxStream;
    /// fn example(mut rx: RxStream) {
    ///     let close = rx.close();
    ///     let _ = rx.start(); // rx was moved into close
    /// }
    /// ```
    ///
    /// A completed close cannot be called again:
    ///
    /// ```compile_fail,E0382
    /// # use hydrasdr_rs::{RawAdc, RxStream};
    /// async fn example(rx: RxStream<RawAdc>) {
    ///     let _ = rx.close().await;
    ///     let _ = rx.close().await; // rx was consumed
    /// }
    /// ```
    pub fn close(self) -> impl MaybeFuture<Output = Result<StreamingStats>> + 'static {
        CloseOperation { state: self.state }
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
        match &mut self.state {
            RxStreamState::AsyncRaw(stream) => stream.start().await,
            RxStreamState::AsyncF32(stream) => stream.start().await,
            _ => Err(Error::Busy),
        }
    }

    async fn stop_async(&mut self) -> Result<StreamingStats> {
        match &mut self.state {
            RxStreamState::Dormant(_) => Ok(StreamingStats::default()),
            RxStreamState::AsyncRaw(stream) => stream.stop().await,
            RxStreamState::AsyncF32(stream) => stream.stop().await,
            #[cfg(not(target_arch = "wasm32"))]
            RxStreamState::BlockingRaw(_) | RxStreamState::BlockingF32(_) => Err(Error::Busy),
            RxStreamState::Poisoned => Err(Error::stream_closed("RX stream has no device")),
        }
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
        match &mut self.state {
            RxStreamState::BlockingRaw(stream) => stream.start(),
            RxStreamState::BlockingF32(stream) => stream.start(),
            _ => Err(Error::Busy),
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn stop_blocking(&mut self) -> Result<StreamingStats> {
        match &mut self.state {
            RxStreamState::Dormant(_) => Ok(StreamingStats::default()),
            RxStreamState::BlockingRaw(stream) => stream.stop(),
            RxStreamState::BlockingF32(stream) => stream.stop(),
            RxStreamState::AsyncRaw(_) | RxStreamState::AsyncF32(_) => Err(Error::Busy),
            RxStreamState::Poisoned => Err(Error::stream_closed("RX stream has no device")),
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
    /// The steady-state asynchronous path uses a concrete future without a
    /// per-call future allocation; changing raw packing may allocate while the
    /// USB queue is rebuilt.
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

    fn async_raw_packing_changed(&self) -> bool {
        match &self.state {
            RxStreamState::AsyncRaw(stream) => stream.packing_changed(),
            _ => false,
        }
    }

    fn poll_next_block_async(&mut self, cx: &mut Context<'_>) -> Poll<Result<()>> {
        match &mut self.state {
            RxStreamState::AsyncRaw(stream) => stream.poll_next_block(cx),
            _ => Poll::Ready(Err(Error::stream_closed(
                "raw RX stream is not running asynchronously",
            ))),
        }
    }

    fn current_async_raw_block(&self) -> Result<Option<SampleBlock<'_>>> {
        match &self.state {
            RxStreamState::AsyncRaw(stream) => stream.current_block().map(Some),
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
    /// operation is awaited. Awaiting this operation uses a concrete future
    /// without a per-call future allocation.
    pub fn read<'a>(
        &'a mut self,
        out: &'a mut [Complex32],
        timeout: Option<Duration>,
    ) -> impl MaybeFuture<Output = Result<usize>> + 'a {
        ReadOperation {
            stream: self,
            out,
            timeout,
            initialized: false,
            completed: false,
        }
    }

    fn poll_read_async(
        &mut self,
        out: &mut [Complex32],
        cx: &mut Context<'_>,
    ) -> Poll<Result<usize>> {
        match &mut self.state {
            RxStreamState::AsyncF32(stream) => stream.poll_read(out, cx),
            _ => Poll::Ready(Err(Error::stream_closed(
                "F32 IQ stream is not running asynchronously",
            ))),
        }
    }

    fn begin_read_async(&mut self) -> Result<()> {
        match &mut self.state {
            RxStreamState::AsyncF32(stream) => stream.begin_read(),
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
type BoxedOperationFuture<'a, T> = Pin<Box<dyn Future<Output = T> + Send + 'a>>;
#[cfg(target_arch = "wasm32")]
type BoxedOperationFuture<'a, T> = Pin<Box<dyn Future<Output = T> + 'a>>;

struct StartOperation<'a, M: SampleMode> {
    stream: &'a mut RxStream<M>,
}

impl<'a, M: SampleMode> IntoFuture for StartOperation<'a, M> {
    type Output = Result<()>;
    type IntoFuture = BoxedOperationFuture<'a, Self::Output>;

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

struct CloseOperation<C: RxBackend> {
    state: RxStreamState<C>,
}

impl<C: RxBackend> IntoFuture for CloseOperation<C>
where
    RxStreamState<C>: crate::maybe_future::NonWasmSend,
{
    type Output = Result<StreamingStats>;
    type IntoFuture = BoxedOperationFuture<'static, Self::Output>;

    fn into_future(self) -> Self::IntoFuture {
        Box::pin(async move {
            let (mut claim, stats) = self.state.into_close_parts()?;
            claim.close().await?;
            Ok(stats)
        })
    }
}

impl<C: RxBackend> MaybeFuture for CloseOperation<C>
where
    RxStreamState<C>: crate::maybe_future::NonWasmSend,
{
    #[cfg(not(target_arch = "wasm32"))]
    fn wait(self) -> Self::Output {
        let (mut claim, stats) = self.state.into_close_parts()?;
        claim.close().wait()?;
        Ok(stats)
    }
}

struct StopOperation<'a, M: SampleMode> {
    stream: &'a mut RxStream<M>,
}

impl<'a, M: SampleMode> IntoFuture for StopOperation<'a, M> {
    type Output = Result<StreamingStats>;
    type IntoFuture = BoxedOperationFuture<'a, Self::Output>;

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
    type IntoFuture = NextBlockFuture<'a>;

    fn into_future(self) -> Self::IntoFuture {
        let _ = self.timeout;
        if self.stream.async_raw_packing_changed() {
            NextBlockFuture::Restarting(Box::pin(self.stream.next_block_async()))
        } else {
            NextBlockFuture::Direct(Some(self.stream))
        }
    }
}

enum NextBlockFuture<'a> {
    Direct(Option<&'a mut RxStream<RawAdc>>),
    Restarting(BoxedOperationFuture<'a, Result<Option<SampleBlock<'a>>>>),
    Transitioning,
}

impl<'a> Future for NextBlockFuture<'a> {
    type Output = Result<Option<SampleBlock<'a>>>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.as_mut().get_mut();
        let packing_changed = match this {
            Self::Direct(stream) => stream
                .as_deref()
                .is_some_and(RxStream::async_raw_packing_changed),
            Self::Restarting(_) | Self::Transitioning => false,
        };
        if packing_changed {
            let Self::Direct(Some(stream)) = core::mem::replace(this, Self::Transitioning) else {
                unreachable!("packing change checked on direct raw stream")
            };
            *this = Self::Restarting(Box::pin(stream.next_block_async()));
        }

        match this {
            Self::Direct(stream) => {
                let stream = stream
                    .take()
                    .expect("next-block future polled after completion");
                match stream.poll_next_block_async(cx) {
                    Poll::Pending => {
                        *this = Self::Direct(Some(stream));
                        Poll::Pending
                    }
                    Poll::Ready(Err(error)) => Poll::Ready(Err(error)),
                    Poll::Ready(Ok(())) => {
                        let stream: &'a RxStream<RawAdc> = stream;
                        Poll::Ready(stream.current_async_raw_block())
                    }
                }
            }
            Self::Restarting(future) => future.as_mut().poll(cx),
            Self::Transitioning => unreachable!("next-block future transition is synchronous"),
        }
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
    initialized: bool,
    completed: bool,
}

impl Future for ReadOperation<'_> {
    type Output = Result<usize>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.as_mut().get_mut();
        assert!(!this.completed, "read future polled after completion");
        let _ = this.timeout;
        if !this.initialized {
            if let Err(error) = this.stream.begin_read_async() {
                this.completed = true;
                return Poll::Ready(Err(error));
            }
            this.initialized = true;
        }
        let result = this.stream.poll_read_async(this.out, cx);
        if result.is_ready() {
            this.completed = true;
        }
        result
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

    #[derive(Debug, Default)]
    struct FakeState {
        fail_packing: AtomicBool,
        control_out_count: AtomicUsize,
        control_out_requests: Mutex<Vec<VendorControlRequest>>,
        fail_control_out: AtomicBool,
        fail_control_out_at: AtomicUsize,
        pause_control_out_at: AtomicUsize,
        pause_clear_halt: AtomicBool,
        fail_bulk_completion: AtomicBool,
        bulk_in_count: AtomicUsize,
        cancel_count: AtomicUsize,
        events: Mutex<Vec<FakeEvent>>,
    }

    #[derive(Clone, Debug, Eq, PartialEq)]
    enum FakeEvent {
        Control(VendorControlRequest),
        QueueDropped,
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
                    .events
                    .lock()
                    .unwrap()
                    .push(FakeEvent::Control(self.request.clone()));
                self.state
                    .control_out_requests
                    .lock()
                    .expect("control request lock")
                    .push(self.request);
                let call = self.state.control_out_count.fetch_add(1, Ordering::SeqCst) + 1;
                // Tests explicitly poll again after clearing the pause.
                std::future::poll_fn(|_| {
                    if self.state.pause_control_out_at.load(Ordering::SeqCst) == call {
                        Poll::Pending
                    } else {
                        Poll::Ready(())
                    }
                })
                .await;
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
                .events
                .lock()
                .unwrap()
                .push(FakeEvent::Control(self.request.clone()));
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
    impl Drop for FakeBulkIn {
        fn drop(&mut self) {
            self.state
                .events
                .lock()
                .unwrap()
                .push(FakeEvent::QueueDropped);
        }
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

    impl Drop for FakeAsyncBulkIn {
        fn drop(&mut self) {
            self.state
                .events
                .lock()
                .unwrap()
                .push(FakeEvent::QueueDropped);
        }
    }

    impl AsyncBulkInBackend for FakeAsyncBulkIn {
        type Buffer = Vec<u8>;

        async fn clear_halt_async(&mut self) -> Result<()> {
            std::future::poll_fn(|_| {
                if self.state.pause_clear_halt.load(Ordering::SeqCst) {
                    Poll::Pending
                } else {
                    Poll::Ready(())
                }
            })
            .await;
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

        fn poll_next_complete(
            &mut self,
            _cx: &mut std::task::Context<'_>,
        ) -> std::task::Poll<BulkInCompletion<Self::Buffer>> {
            let buffer = self
                .submitted
                .pop_front()
                .expect("fake async bulk queue contains submitted buffers");
            std::task::Poll::Ready(BulkInCompletion {
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

    fn fake_device(control: FakeControl) -> DeviceInner<FakeControl> {
        DeviceInner::new(
            HydraSdr::from_control(control),
            DeviceInfo {
                board_name: "fake HydraSDR",
                firmware_version: "fake firmware".to_owned(),
                serial: None,
                rf_ports: Vec::new(),
            },
        )
    }

    fn owned_stream_claim(device: DeviceInner<FakeControl>) -> RxStreamClaim<FakeControl> {
        RxStreamClaim::acquire(&device.shared).expect("claim stream before releasing device")
    }

    fn requests(state: &FakeState) -> Vec<VendorControlRequest> {
        state.control_out_requests.lock().unwrap().clone()
    }

    #[test]
    fn dormant_claim_keeps_device_alive_until_its_drop() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        let device = fake_device(control);
        let owner = Arc::downgrade(&device.shared);
        let claim = RxStreamClaim::acquire(&device.shared).unwrap();
        assert!(matches!(
            RxStreamClaim::acquire(&device.shared),
            Err(Error::Busy)
        ));

        drop(device);
        assert!(owner.upgrade().is_some());
        assert!(requests(&state).is_empty());

        drop(claim);
        assert!(owner.upgrade().is_none());
        assert_eq!(
            requests(&state),
            [
                VendorControlRequest::receiver_mode(ReceiverMode::Off),
                VendorControlRequest::set_rf_bias(0),
            ]
        );
    }

    #[test]
    fn dropping_a_dormant_claim_allows_a_replacement_without_usb_commands() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        let device = fake_device(control);
        drop(RxStreamClaim::acquire(&device.shared).unwrap());
        let _replacement = RxStreamClaim::acquire(&device.shared).unwrap();
        assert!(requests(&state).is_empty());
    }

    #[test]
    fn async_stream_can_start_read_and_restart_after_device_drop() {
        block_on(async {
            // Drop the public owner before starting, while running, or after stop.
            for phase in 0..3 {
                let control = FakeControl::default();
                let state = Arc::clone(&control.state);
                let mut device = fake_device(control);
                device.set_bias_tee(true).await.unwrap();
                let owner = Arc::downgrade(&device.shared);
                let mut stream =
                    AsyncF32RxStreamInner::new(RxStreamClaim::acquire(&device.shared).unwrap());
                if phase >= 1 {
                    stream.start().await.unwrap();
                }
                if phase >= 2 {
                    stream.stop().await.unwrap();
                }
                let before_drop = requests(&state);
                drop(device);
                assert_eq!(requests(&state), before_drop);

                let mut sample = [Complex32::default(); 1];
                for _ in 0..2 {
                    stream.start().await.unwrap();
                    assert_eq!(stream.read(&mut sample).await.unwrap(), 1);
                    stream.stop().await.unwrap();
                }
                assert!(owner.upgrade().is_some());
                assert!(!requests(&state).contains(&VendorControlRequest::set_rf_bias(0)));
                drop(stream);
                assert!(owner.upgrade().is_none());
                assert!(requests(&state).ends_with(&[
                    VendorControlRequest::receiver_mode(ReceiverMode::Off),
                    VendorControlRequest::set_rf_bias(0),
                ]));
            }
        });
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn blocking_raw_stream_can_restart_after_device_drop() {
        for phase in 0..3 {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control);
            let mut stream =
                OwnedRawRxStreamInner::new(RxStreamClaim::acquire(&device.shared).unwrap());
            if phase >= 1 {
                stream.start().unwrap();
            }
            if phase >= 2 {
                stream.stop().unwrap();
            }
            let before_drop = requests(&state);
            drop(device);
            assert_eq!(requests(&state), before_drop);
            for _ in 0..2 {
                stream.start().unwrap();
                assert!(stream.next_block(Duration::MAX).unwrap().is_some());
                stream.stop().unwrap();
            }
            assert!(!requests(&state).contains(&VendorControlRequest::set_rf_bias(0)));
            stream.start().unwrap();
            drop(stream);
            assert!(state.events.lock().unwrap().ends_with(&[
                FakeEvent::QueueDropped,
                FakeEvent::Control(VendorControlRequest::receiver_mode(ReceiverMode::Off)),
                FakeEvent::Control(VendorControlRequest::receiver_mode(ReceiverMode::Off)),
                FakeEvent::Control(VendorControlRequest::set_rf_bias(0)),
            ]));
            assert_eq!(
                requests(&state).last(),
                Some(&VendorControlRequest::set_rf_bias(0))
            );
        }
    }

    #[test]
    fn dropping_an_active_stream_allows_a_replacement_and_leaves_device_open() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            let mut device = fake_device(control);
            let mut stream =
                AsyncRawRxStreamInner::new(RxStreamClaim::acquire(&device.shared).unwrap());
            stream.start().await.unwrap();
            drop(stream);
            assert!(state.events.lock().unwrap().ends_with(&[
                FakeEvent::QueueDropped,
                FakeEvent::Control(VendorControlRequest::receiver_mode(ReceiverMode::Off)),
            ]));
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
            assert_eq!(
                requests(&state),
                [
                    VendorControlRequest::receiver_mode(ReceiverMode::Off),
                    VendorControlRequest::receiver_mode(ReceiverMode::Rx),
                    VendorControlRequest::receiver_mode(ReceiverMode::Off),
                ]
            );
            device.shared.ensure_open().unwrap();
            let mut replacement =
                AsyncRawRxStreamInner::new(RxStreamClaim::acquire(&device.shared).unwrap());
            replacement.start().await.unwrap();
            replacement.stop().await.unwrap();
            replacement.retire_stream();
            replacement.claim.close().await.unwrap();
            device.shutdown().await.unwrap();
            let after_shutdown = requests(&state);
            drop(device);
            drop(replacement);
            assert_eq!(requests(&state), after_shutdown);
        });
    }

    async fn close_test_stream(
        device: &DeviceInner<FakeControl>,
        mode: usize,
        phase: usize,
    ) -> RxStreamState<FakeControl> {
        let claim = RxStreamClaim::acquire(&device.shared).unwrap();
        if phase == 0 {
            return RxStreamState::Dormant(claim);
        }
        match mode {
            0 => {
                let mut inner = OwnedRawRxStreamInner::new(claim);
                inner.start().unwrap();
                inner.next_block(Duration::from_secs(1)).unwrap();
                if phase == 2 {
                    inner.stop().unwrap();
                }
                RxStreamState::BlockingRaw(inner)
            }
            1 => {
                let mut inner = F32RxStreamInner::new(claim);
                inner.start().unwrap();
                inner
                    .read(&mut [Complex32::default(); 1], Duration::from_secs(1))
                    .unwrap();
                if phase == 2 {
                    inner.stop().unwrap();
                }
                RxStreamState::BlockingF32(Box::new(inner))
            }
            2 => {
                let mut inner = AsyncRawRxStreamInner::new(claim);
                inner.start().await.unwrap();
                inner.next_block().await.unwrap();
                if phase == 2 {
                    inner.stop().await.unwrap();
                }
                RxStreamState::AsyncRaw(inner)
            }
            _ => {
                let mut inner = AsyncF32RxStreamInner::new(claim);
                inner.start().await.unwrap();
                inner.read(&mut [Complex32::default(); 1]).await.unwrap();
                if phase == 2 {
                    inner.stop().await.unwrap();
                }
                RxStreamState::AsyncF32(Box::new(inner))
            }
        }
    }

    #[test]
    fn close_all_modes_releases_queue_claim_and_owner_and_returns_statistics() {
        block_on(async {
            for mode in 0..4 {
                for phase in 0..3 {
                    for wait in [false, true] {
                        let control = FakeControl::default();
                        let state = Arc::clone(&control.state);
                        let mut device = fake_device(control);
                        let rx = close_test_stream(&device, mode, phase).await;
                        assert!(matches!(device.shutdown().await, Err(Error::Busy)));
                        let before = requests(&state);
                        let operation = CloseOperation { state: rx };
                        assert_eq!(requests(&state), before);
                        let mut completed = None;
                        let stats = if wait {
                            operation.wait().unwrap()
                        } else {
                            let mut future = operation.into_future();
                            let stats = future.as_mut().await.unwrap();
                            completed = Some(future);
                            stats
                        };
                        assert_eq!(Arc::strong_count(&device.shared), 1);
                        assert_eq!(
                            requests(&state).len(),
                            before.len() + usize::from(phase == 1)
                        );
                        if phase == 0 {
                            assert_eq!(stats, StreamingStats::default());
                        } else {
                            assert!(stats.buffers_received > 0);
                            assert_eq!(
                                state
                                    .events
                                    .lock()
                                    .unwrap()
                                    .iter()
                                    .filter(|event| **event == FakeEvent::QueueDropped)
                                    .count(),
                                1
                            );
                        }
                        let after = requests(&state);
                        let replacement = RxStreamClaim::acquire(&device.shared).unwrap();
                        // Even a retained completed future has released its ownership.
                        drop(completed);
                        assert!(matches!(
                            RxStreamClaim::acquire(&device.shared),
                            Err(Error::Busy)
                        ));
                        assert_eq!(requests(&state), after);
                        drop(replacement);
                        device.shutdown().await.unwrap();
                    }
                }
            }
        });
    }

    #[test]
    fn dropping_unpolled_close_operation_or_future_cleans_up_its_owned_stream() {
        block_on(async {
            for mode in 0..4 {
                for phase in 0..3 {
                    for into_future in [false, true] {
                        let control = FakeControl::default();
                        let state = Arc::clone(&control.state);
                        let mut device = fake_device(control);
                        let rx = close_test_stream(&device, mode, phase).await;
                        let before = state.events.lock().unwrap().clone();
                        let operation = CloseOperation { state: rx };
                        assert_eq!(*state.events.lock().unwrap(), before);
                        assert!(matches!(device.shutdown().await, Err(Error::Busy)));
                        if into_future {
                            let future = operation.into_future();
                            assert_eq!(*state.events.lock().unwrap(), before);
                            assert!(matches!(device.shutdown().await, Err(Error::Busy)));
                            drop(future);
                        } else {
                            drop(operation);
                        }
                        assert_eq!(Arc::strong_count(&device.shared), 1);
                        if phase == 1 {
                            assert!(state.events.lock().unwrap().ends_with(&[
                                FakeEvent::QueueDropped,
                                FakeEvent::Control(VendorControlRequest::receiver_mode(
                                    ReceiverMode::Off
                                )),
                            ]));
                        }
                        let replacement = RxStreamClaim::acquire(&device.shared).unwrap();
                        drop(replacement);
                        device.shutdown().await.unwrap();
                    }
                }
            }
        });
    }

    #[test]
    fn canceled_close_retains_claim_until_drop_fallback_finishes() {
        block_on(async {
            for mode in 0..4 {
                for fail_fallback in [false, true] {
                    let control = FakeControl::default();
                    let state = Arc::clone(&control.state);
                    let mut device = fake_device(control);
                    let rx = close_test_stream(&device, mode, 1).await;
                    state
                        .pause_control_out_at
                        .store(requests(&state).len() + 1, Ordering::SeqCst);
                    let mut close = CloseOperation { state: rx }.into_future();
                    assert!(futures_lite::future::poll_once(&mut close).await.is_none());
                    assert!(matches!(device.shutdown().await, Err(Error::Busy)));
                    assert!(matches!(
                        RxStreamClaim::acquire(&device.shared),
                        Err(Error::Busy)
                    ));
                    assert!(state.events.lock().unwrap().ends_with(&[
                        FakeEvent::QueueDropped,
                        FakeEvent::Control(VendorControlRequest::receiver_mode(ReceiverMode::Off)),
                    ]));
                    state
                        .fail_control_out
                        .store(fail_fallback, Ordering::SeqCst);
                    drop(close);
                    assert_eq!(Arc::strong_count(&device.shared), 1);
                    assert_eq!(
                        device.shared.lock().claim,
                        if fail_fallback {
                            ClaimAvailability::CleanupRequired
                        } else {
                            ClaimAvailability::Available
                        }
                    );
                    if fail_fallback {
                        assert!(matches!(
                            RxStreamClaim::acquire(&device.shared),
                            Err(Error::Busy)
                        ));
                    } else {
                        drop(RxStreamClaim::acquire(&device.shared).unwrap());
                    }
                    state.fail_control_out.store(false, Ordering::SeqCst);
                    state.pause_control_out_at.store(0, Ordering::SeqCst);
                    device.shutdown().await.unwrap();
                }
            }
        });
    }

    #[test]
    fn failed_close_returns_original_error_and_runs_drop_fallback() {
        block_on(async {
            for mode in 0..4 {
                for wait in [false, true] {
                    for fail_fallback in [false, true] {
                        let control = FakeControl::default();
                        let state = Arc::clone(&control.state);
                        let mut device = fake_device(control);
                        let rx = close_test_stream(&device, mode, 1).await;
                        let before = requests(&state).len();
                        state
                            .fail_control_out_at
                            .store(before + 1, Ordering::SeqCst);
                        state
                            .fail_control_out
                            .store(fail_fallback, Ordering::SeqCst);
                        let operation = CloseOperation { state: rx };
                        let error = if wait {
                            operation.wait().unwrap_err()
                        } else {
                            operation.await.unwrap_err()
                        };
                        assert!(matches!(
                            error,
                            Error::Transfer(nusb::transfer::TransferError::Fault)
                        ));
                        assert_eq!(requests(&state).len(), before + 2);
                        assert_eq!(Arc::strong_count(&device.shared), 1);
                        assert_eq!(
                            device.shared.lock().claim,
                            if fail_fallback {
                                ClaimAvailability::CleanupRequired
                            } else {
                                ClaimAvailability::Available
                            }
                        );
                        state.fail_control_out.store(false, Ordering::SeqCst);
                        device.shutdown().await.unwrap();
                    }
                }
            }
        });
    }

    #[test]
    fn close_after_failed_stop_and_device_drop_completes_final_owner_cleanup() {
        block_on(async {
            for mode in 0..4 {
                let control = FakeControl::default();
                let state = Arc::clone(&control.state);
                let device = fake_device(control);
                let owner = Arc::downgrade(&device.shared);
                let mut rx = close_test_stream(&device, mode, 1).await;
                state.fail_control_out.store(true, Ordering::SeqCst);
                let result = match &mut rx {
                    RxStreamState::BlockingRaw(inner) => inner.stop(),
                    RxStreamState::BlockingF32(inner) => inner.stop(),
                    RxStreamState::AsyncRaw(inner) => inner.stop().await,
                    RxStreamState::AsyncF32(inner) => inner.stop().await,
                    _ => unreachable!(),
                };
                assert!(result.is_err());
                state.fail_control_out.store(false, Ordering::SeqCst);
                drop(device);
                assert!(owner.upgrade().is_some());
                let stats = CloseOperation { state: rx }.await.unwrap();
                assert!(stats.buffers_received > 0);
                assert!(owner.upgrade().is_none());
                assert!(requests(&state).ends_with(&[
                    VendorControlRequest::receiver_mode(ReceiverMode::Off),
                    VendorControlRequest::receiver_mode(ReceiverMode::Off),
                    VendorControlRequest::set_rf_bias(0),
                ]));
                assert_eq!(Arc::strong_count(&state), 1);
            }
        });
    }

    #[test]
    fn public_close_operation_and_future_are_send_and_static() {
        fn assert_owned<T: Send + 'static>(_: &T) {}
        fn check<M: SampleMode>() {
            // No device is needed to check the public operation's ownership bounds.
            let rx = RxStream::<M> {
                state: RxStreamState::Poisoned,
                mode: PhantomData,
            };
            let operation = rx.close();
            assert_owned(&operation);
            let future = operation.into_future();
            assert_owned(&future);
        }
        check::<RawAdc>();
        check::<F32Iq>();
    }

    #[test]
    fn shutdown_requires_release_even_for_dormant_and_stopped_streams() {
        block_on(async {
            for started in [false, true] {
                let control = FakeControl::default();
                let state = Arc::clone(&control.state);
                let mut device = fake_device(control);
                let mut stream =
                    AsyncF32RxStreamInner::new(RxStreamClaim::acquire(&device.shared).unwrap());
                let before = requests(&state);
                drop(device.shutdown());
                device.shared.ensure_open().unwrap();
                assert_eq!(requests(&state), before);
                assert!(matches!(device.shutdown().await, Err(Error::Busy)));
                if started {
                    stream.start().await.unwrap();
                    assert!(matches!(device.shutdown().await, Err(Error::Busy)));
                    stream.stop().await.unwrap();
                }
                assert!(matches!(device.shutdown().await, Err(Error::Busy)));
                stream.retire_stream();
                stream.claim.close().await.unwrap();
                device.shutdown().await.unwrap();
                device.shutdown().await.unwrap();
                let after = requests(&state);
                drop(stream);
                drop(device);
                assert_eq!(requests(&state), after);
            }
        });
    }

    #[test]
    fn failed_shutdown_remains_closing_and_can_be_retried() {
        block_on(async {
            for failed_command in [1, 2] {
                let control = FakeControl::default();
                let state = Arc::clone(&control.state);
                state
                    .fail_control_out_at
                    .store(failed_command, Ordering::SeqCst);
                let mut device = fake_device(control);
                assert!(device.shutdown().await.is_err());
                assert_eq!(device.shared.lock().device, DeviceLifecycle::Closing);
                assert_eq!(requests(&state).len(), 2);
                assert!(matches!(
                    RxStreamClaim::acquire(&device.shared),
                    Err(Error::DeviceClosed)
                ));
                device.shutdown().await.unwrap();
                assert_eq!(device.shared.lock().device, DeviceLifecycle::Closed);
                drop(device);
                assert_eq!(requests(&state).len(), 4);
            }
        });
    }

    #[test]
    fn failed_stream_drop_cleanup_blocks_replacement_until_device_shutdown() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            let mut device = fake_device(control);
            let mut stream =
                AsyncF32RxStreamInner::new(RxStreamClaim::acquire(&device.shared).unwrap());
            stream.start().await.unwrap();
            state.fail_control_out_at.store(3, Ordering::SeqCst);
            drop(stream);
            // Only the claim tries receiver cleanup; there is no inner retry.
            assert_eq!(requests(&state).len(), 3);
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
            assert_ne!(device.shared.lock().claim, ClaimAvailability::Held);
            assert_eq!(
                device.shared.lock().claim,
                ClaimAvailability::CleanupRequired
            );
            assert!(matches!(
                RxStreamClaim::acquire(&device.shared),
                Err(Error::Busy)
            ));
            device.shutdown().await.unwrap();
            assert_eq!(requests(&state).len(), 5);
            drop(device);
            assert_eq!(requests(&state).len(), 5);
        });
    }

    #[test]
    fn cancelled_async_stop_requires_cleanup_before_read_restart_or_shutdown() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            let mut device = fake_device(control);
            let mut stream =
                AsyncF32RxStreamInner::new(RxStreamClaim::acquire(&device.shared).unwrap());
            stream.start().await.unwrap();
            state.pause_control_out_at.store(3, Ordering::SeqCst);
            let mut stop = Box::pin(stream.stop());
            assert!(futures_lite::future::poll_once(&mut stop).await.is_none());
            drop(stop);
            let mut sample = [Complex32::default(); 1];
            assert!(stream.read(&mut sample).await.is_err());
            assert!(matches!(stream.start().await, Err(Error::Busy)));
            assert!(matches!(device.shutdown().await, Err(Error::Busy)));
            assert_eq!(requests(&state).len(), 3);
            stream.stop().await.unwrap();
            stream.start().await.unwrap();
            assert_eq!(stream.read(&mut sample).await.unwrap(), 1);
            stream.stop().await.unwrap();
            drop(stream);
            device.shutdown().await.unwrap();
        });
    }

    #[test]
    fn device_drop_during_async_start_does_not_interrupt_or_shut_down_the_stream() {
        block_on(async {
            // Pause during initial receiver-off, receiver-on, or queue setup.
            for phase in 1..=3 {
                let control = FakeControl::default();
                let state = Arc::clone(&control.state);
                let device = fake_device(control);
                let owner = Arc::downgrade(&device.shared);
                let mut stream =
                    AsyncF32RxStreamInner::new(RxStreamClaim::acquire(&device.shared).unwrap());
                if phase < 3 {
                    state.pause_control_out_at.store(phase, Ordering::SeqCst);
                } else {
                    state.pause_clear_halt.store(true, Ordering::SeqCst);
                }
                let mut start = Box::pin(stream.start());
                assert!(futures_lite::future::poll_once(&mut start).await.is_none());
                let before_drop = requests(&state);
                drop(device);
                assert!(owner.upgrade().is_some());
                assert_eq!(requests(&state), before_drop);
                drop(start);
                assert!(matches!(stream.start().await, Err(Error::Busy)));
                stream.stop().await.unwrap();
                state.pause_control_out_at.store(0, Ordering::SeqCst);
                state.pause_clear_halt.store(false, Ordering::SeqCst);
                stream.start().await.unwrap();
                let mut sample = [Complex32::default(); 1];
                assert_eq!(stream.read(&mut sample).await.unwrap(), 1);
                drop(stream);
                assert!(owner.upgrade().is_none());
                assert!(state.events.lock().unwrap().ends_with(&[
                    FakeEvent::QueueDropped,
                    FakeEvent::Control(VendorControlRequest::receiver_mode(ReceiverMode::Off)),
                    FakeEvent::Control(VendorControlRequest::receiver_mode(ReceiverMode::Off)),
                    FakeEvent::Control(VendorControlRequest::set_rf_bias(0)),
                ]));
            }
        });
    }

    #[test]
    fn background_cleanup_reserves_the_claim_until_receiver_off_completes() {
        block_on(async {
            for fail in [false, true] {
                let control = FakeControl::default();
                let state = Arc::clone(&control.state);
                let mut device = fake_device(control);
                let mut stream =
                    AsyncRawRxStreamInner::new(RxStreamClaim::acquire(&device.shared).unwrap());
                stream.start().await.unwrap();
                // Drive the same owned operation that Drop schedules on WebUSB.
                stream.retire_stream();
                state.pause_control_out_at.store(3, Ordering::SeqCst);
                let mut cleanup = Box::pin(stream.claim.cleanup().into_future());
                assert!(matches!(device.shutdown().await, Err(Error::Busy)));
                assert!(
                    futures_lite::future::poll_once(&mut cleanup)
                        .await
                        .is_none()
                );
                assert!(matches!(
                    RxStreamClaim::acquire(&device.shared),
                    Err(Error::Busy)
                ));
                assert!(matches!(device.shutdown().await, Err(Error::Busy)));
                state.fail_control_out.store(fail, Ordering::SeqCst);
                state.pause_control_out_at.store(0, Ordering::SeqCst);
                cleanup.await;
                assert_ne!(device.shared.lock().claim, ClaimAvailability::Held);
                assert_eq!(
                    device.shared.lock().claim,
                    if fail {
                        ClaimAvailability::CleanupRequired
                    } else {
                        ClaimAvailability::Available
                    }
                );
                if fail {
                    assert!(matches!(
                        RxStreamClaim::acquire(&device.shared),
                        Err(Error::Busy)
                    ));
                }
                // Retire the test's source claim before acquiring another one.
                drop(stream);
                state.fail_control_out.store(false, Ordering::SeqCst);
                if !fail {
                    drop(RxStreamClaim::acquire(&device.shared).unwrap());
                }
                device.shutdown().await.unwrap();
            }
        });
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
        assert!(device.shared.direct.streaming_packing_enabled());
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
        assert_eq!(device.shared.direct.streaming_decimation_factor(), 2);
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
            let mut stream = OwnedRawRxStreamInner::new(owned_stream_claim(device));
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
        let mut stream = F32RxStreamInner::new(owned_stream_claim(device));
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
        let mut stream = F32RxStreamInner::new(owned_stream_claim(device));
        let mut sample = [Complex32::default(); 1];

        stream.start().expect("start synchronous stream");
        state.fail_bulk_completion.store(true, Ordering::SeqCst);
        assert!(stream.read(&mut sample, Duration::MAX).is_err());
        assert_eq!(
            stream.claim.receiver_state(),
            ReceiverState::CleanupRequired
        );
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
        let mut stream = OwnedRawRxStreamInner::new(owned_stream_claim(device));

        stream.start().expect("start synchronous raw stream");
        state.fail_bulk_completion.store(true, Ordering::SeqCst);
        assert!(stream.next_block(Duration::MAX).is_err());
        assert_eq!(
            stream.claim.receiver_state(),
            ReceiverState::CleanupRequired
        );
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
        let mut stream = F32RxStreamInner::new(owned_stream_claim(device));
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
            let mut stream = AsyncF32RxStreamInner::new(owned_stream_claim(device));
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
            let mut stream = AsyncF32RxStreamInner::new(owned_stream_claim(device));

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
            let mut stream = AsyncF32RxStreamInner::new(owned_stream_claim(device));
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
            let mut stream = AsyncRawRxStreamInner::new(owned_stream_claim(device));

            stream.start().await.expect("start async raw stream");
            state.fail_bulk_completion.store(true, Ordering::SeqCst);
            assert!(stream.next_block().await.is_err());
            assert_eq!(
                stream.claim.receiver_state(),
                ReceiverState::CleanupRequired
            );
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
            let mut stream = AsyncF32RxStreamInner::new(owned_stream_claim(device));
            let mut sample = [Complex32::default(); 1];

            stream.start().await.expect("start async F32 stream");
            state.fail_bulk_completion.store(true, Ordering::SeqCst);
            assert!(stream.read(&mut sample).await.is_err());
            assert_eq!(
                stream.claim.receiver_state(),
                ReceiverState::CleanupRequired
            );
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
            let mut stream = AsyncRawRxStreamInner::new(owned_stream_claim(device));

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
            let mut stream = AsyncF32RxStreamInner::new(owned_stream_claim(device));

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
            let mut stream = AsyncF32RxStreamInner::new(owned_stream_claim(device));
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
            let mut stream = AsyncF32RxStreamInner::new(owned_stream_claim(device));
            stream.start().await.expect("start owned async F32 stream");
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);

            drop(stream);
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 5);
        });
    }
}
