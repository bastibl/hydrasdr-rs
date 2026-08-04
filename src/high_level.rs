//! HydraSDR RFOne sync and async APIs.

use nusb::MaybeFuture;
use std::future::{Future, IntoFuture};
use std::pin::Pin;

use crate::Complex32;
use crate::commands::ReceiverMode;
use crate::config::{
    Bandwidth, Config, ConfigBuilder, ConfigData, DeviceSelector, F32Iq, GainConfig, RawAdc,
    RfPort, SampleFormat, SampleMode, validate_bandwidth, validate_frequency, validate_gain,
    validate_sample_rate,
};
use crate::device::HydraSdr;
use crate::errors::{Error, Result};
use crate::maybe_future::{Either, MaybeFutureExt, ready};
use core::marker::PhantomData;
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
///     let dev = Device::builder()
///         .frequency_hz(100_000_000)
///         .sample_rate_hz(10_000_000)
///         .raw_adc()
///         .rf_port(RfPort::Rx0)
///         .gain(GainPreset::Linearity(12))
///         .open()
///         .wait()?;
///
///     let mut rx = dev.into_rx_stream();
///     rx.start().wait()?;
///     if let Some(block) = rx.next_block(Duration::from_secs(1)).wait()? {
///         println!("{} raw bytes", block.raw_bytes().len());
///     }
///     let stats = rx.stop().wait()?;
///     println!("{stats:?}");
///     rx.shutdown().wait()?;
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

/// High-level owned HydraSDR RFOne device handle.
#[derive(Debug)]
pub struct Device<M: SampleMode = F32Iq> {
    inner: DeviceInner<NusbControl>,
    mode: PhantomData<fn() -> M>,
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
    /// Return cached device metadata.
    pub fn info(&self) -> &DeviceInfo {
        self.inner.info()
    }

    /// Return a shared handle to the authoritative active receiver state.
    pub fn active_state(&self) -> crate::ActiveState {
        self.info().active_state.clone()
    }

    /// Refresh and return device metadata.
    pub fn refresh_info(&mut self) -> impl MaybeFuture<Output = Result<&DeviceInfo>> {
        self.inner.refresh_info()
    }

    /// Return the cached sample rates advertised for the active sample format.
    ///
    /// Converted F32 IQ results include exactly representable effective rates
    /// produced by host-side decimation. The table is fetched while applying
    /// the active configuration, so this accessor performs no USB requests.
    pub fn sample_rates(&self) -> Vec<u32> {
        self.inner.sample_rates()
    }

    /// Query advertised analog bandwidths.
    ///
    /// Current RFOne firmware does not advertise manual bandwidth control and
    /// returns [`Error::Unsupported`].
    pub fn bandwidths(&mut self) -> impl MaybeFuture<Output = Result<Vec<u32>>> {
        self.inner.bandwidths()
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
    pub fn configure(&mut self, config: &Config<M>) -> impl MaybeFuture<Output = Result<()>> {
        self.inner.configure(&config.data)
    }

    /// Set only the tuned center frequency.
    pub fn set_frequency_hz(&mut self, frequency_hz: u64) -> impl MaybeFuture<Output = Result<()>> {
        self.inner.set_frequency_hz(frequency_hz)
    }

    /// Set only the requested sample rate.
    ///
    /// The value is a real ADC rate for [`SampleFormat::RawAdc`] and an
    /// effective complex output rate for [`SampleFormat::F32Iq`]. Values not
    /// returned by [`Device::sample_rates`] are left for firmware to accept or
    /// reject.
    pub fn set_sample_rate_hz(
        &mut self,
        sample_rate_hz: u32,
    ) -> impl MaybeFuture<Output = Result<()>> {
        self.inner.set_sample_rate_hz(sample_rate_hz)
    }

    /// Set only the manual analog bandwidth on firmware that advertises it.
    ///
    /// Current RFOne firmware returns [`Error::Unsupported`].
    pub fn set_bandwidth_hz(&mut self, bandwidth_hz: u32) -> impl MaybeFuture<Output = Result<()>> {
        self.inner.set_bandwidth_hz(bandwidth_hz)
    }

    /// Set only the selected RF input port.
    pub fn set_rf_port(&mut self, port: RfPort) -> impl MaybeFuture<Output = Result<()>> {
        self.inner.set_rf_port(port)
    }

    /// Apply only the supplied gain update.
    pub fn set_gain(&mut self, gain: GainConfig) -> impl MaybeFuture<Output = Result<()>> {
        self.inner.set_gain(gain)
    }

    /// Consume the device after turning off reception and RF input bias power.
    ///
    /// Both shutdown commands are attempted even if turning off the receiver
    /// fails. Await this operation in async code, or call [`MaybeFuture::wait`]
    /// on native targets. Native drops perform the same sequence best-effort;
    /// WebUSB callers must explicitly await shutdown because `Drop` cannot run
    /// asynchronous USB operations. On native targets, dropping or canceling
    /// the returned operation keeps that best-effort fallback armed. WebUSB
    /// callers must poll it to completion to guarantee both commands are
    /// attempted.
    #[must_use = "shutdown must be awaited or waited to send hardware cleanup commands"]
    pub fn shutdown(self) -> impl MaybeFuture<Output = Result<()>> {
        self.inner.shutdown()
    }

    /// Consume the device and create its typed receive stream.
    ///
    /// The stream starts lazily when [`RxStream::start`] is waited or awaited.
    pub fn into_rx_stream(self) -> RxStream<M> {
        RxStream::new(self.inner)
    }
}

impl<C: ControlBackend> DeviceInner<C> {
    /// Return cached device metadata.
    pub(crate) fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn ensure_raw_adc_stream_format(&self) -> Result<()> {
        let state = &self.info.active_state;
        if state.sample_format()? != SampleFormat::RawAdc {
            return Err(Error::invalid_config(
                "sample_format",
                "raw block streams require SampleFormat::RawAdc",
            ));
        }
        state.sample_rate_hz()?;
        state.packing()?;
        Ok(())
    }

    fn ensure_f32_iq_stream_format(&self) -> Result<()> {
        let state = &self.info.active_state;
        if state.sample_format()? != SampleFormat::F32Iq {
            return Err(Error::invalid_config(
                "sample_format",
                "F32 IQ streams require SampleFormat::F32Iq",
            ));
        }
        state.sample_rate_hz()?;
        state.decimation_mode()?;
        state.packing()?;
        Ok(())
    }
}

impl<C> DeviceInner<C>
where
    C: ControlBackend,
{
    fn shutdown_operation(&self) -> impl MaybeFuture<Output = Result<()>> + use<C> {
        let active_state = self.info.active_state.clone();
        active_state.begin_bias_tee_update();

        let receiver_off = self
            .direct
            .receiver_mode(ReceiverMode::Off)
            .map_err(|error| error.at("turning off the receiver during shutdown"));
        let bias_off = self
            .direct
            .set_rf_bias(false)
            .map_err(|error| error.at("turning off RF bias power during shutdown"))
            .map(move |result| {
                active_state.set_bias_tee_result(false, result.is_ok());
                result
            });

        receiver_off
            .map(Ok::<_, Error>)
            .and_then(move |receiver_result| {
                bias_off.map(move |bias_result| receiver_result.and(bias_result))
            })
    }

    fn shutdown(self) -> impl MaybeFuture<Output = Result<()>> + use<C> {
        #[cfg(not(target_arch = "wasm32"))]
        {
            let mut device = self;
            let operation = device.shutdown_operation();
            operation.map(move |result| {
                if result.is_ok() {
                    device.shutdown_on_drop = false;
                }
                drop(device);
                result
            })
        }
        #[cfg(target_arch = "wasm32")]
        {
            self.shutdown_operation()
        }
    }

    /// Apply a high-level receiver configuration through the direct layer.
    pub(crate) fn configure(
        &mut self,
        config: &ConfigData,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let active_state = self.info.active_state.clone();
        active_state.begin_config(config);
        let operation = self.direct.configure(config);
        let config = config.clone();
        operation.map(move |result| match result {
            Ok(()) => {
                active_state.apply_config(&config);
                Ok(())
            }
            Err(error) => {
                active_state.fail_config(&config);
                Err(error)
            }
        })
    }

    fn sample_rates(&self) -> Vec<u32> {
        self.direct.visible_sample_rates()
    }

    fn bandwidths(&mut self) -> impl MaybeFuture<Output = Result<Vec<u32>>> + use<'_, C> {
        self.direct.get_bandwidths()
    }

    fn set_frequency_hz(
        &mut self,
        frequency_hz: u64,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let active_state = self.info.active_state.clone();
        active_state.begin_frequency_update();
        let operation = self.direct.set_freq(frequency_hz);
        ready(validate_frequency(frequency_hz))
            .and_then(move |()| operation)
            .map(move |result| {
                active_state.set_frequency_result(frequency_hz, result.is_ok());
                result
            })
    }

    fn set_sample_rate_hz(
        &mut self,
        sample_rate_hz: u32,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let active_state = self.info.active_state.clone();
        active_state.begin_sample_rate_update();
        let validation = self
            .info
            .active_state
            .sample_format()
            .and_then(|sample_format| validate_sample_rate(sample_rate_hz, sample_format));
        let operation = self.direct.set_samplerate(sample_rate_hz);
        ready(validation)
            .and_then(move |()| operation)
            .map(move |result| {
                active_state.set_sample_rate_result(sample_rate_hz, result.is_ok());
                result
            })
    }

    fn set_bandwidth_hz(
        &mut self,
        bandwidth_hz: u32,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let active_state = self.info.active_state.clone();
        active_state.begin_bandwidth_update();
        let bandwidth = Bandwidth::ManualHz(bandwidth_hz);
        let operation = self.direct.set_bandwidth(bandwidth_hz);
        ready(validate_bandwidth(bandwidth))
            .and_then(move |()| operation)
            .map(move |result| {
                active_state.set_bandwidth_result(bandwidth_hz, result.is_ok());
                result
            })
    }

    fn set_rf_port(&mut self, port: RfPort) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let active_state = self.info.active_state.clone();
        active_state.begin_rf_port_update();
        let operation = self.direct.set_rf_port(port);
        operation.map(move |result| {
            active_state.set_rf_port_result(port, result.is_ok());
            result
        })
    }

    fn set_gain(&mut self, gain: GainConfig) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let active_state = self.info.active_state.clone();
        active_state.begin_gain_update(gain);
        let operation = self.direct.set_gain_config(gain);
        ready(validate_gain(gain))
            .and_then(move |()| operation)
            .map(move |result| {
                active_state.set_gain_result(gain, result.is_ok());
                result
            })
    }

    /// Refresh and return direct device metadata.
    pub(crate) fn refresh_info(
        &mut self,
    ) -> impl MaybeFuture<Output = Result<&DeviceInfo>> + use<'_, C> {
        let active_state = self.info.active_state.clone();
        let operation = self.direct.get_device_info();
        let info_slot = &mut self.info;
        operation.map(move |result| {
            let mut info = result?;
            info.active_state = active_state;
            *info_slot = info;
            Ok(&*info_slot)
        })
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
///     .sample_rate_hz(2_000_000)
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
    /// host-side decimation. The builder validates only USB protocol encoding,
    /// not firmware support. Query [`Device::sample_rates`] on an opened device
    /// for its advertised rates.
    pub fn sample_rate_hz(mut self, value: u32) -> Self {
        self.config = self.config.sample_rate_hz(value);
        self
    }

    /// Set the analog bandwidth policy.
    ///
    /// [`crate::Bandwidth::ManualHz`] is capability-gated. Its numeric bound
    /// reflects the vendor request encoding, not an RFOne hardware range;
    /// current RFOne firmware does not advertise manual bandwidth control.
    pub fn bandwidth(mut self, value: crate::Bandwidth) -> Self {
        self.config = self.config.bandwidth(value);
        self
    }

    /// Set an explicit analog bandwidth in Hz.
    ///
    /// This is shorthand for [`DeviceBuilder::bandwidth`] with
    /// [`crate::Bandwidth::ManualHz`]. The value must fit the vendor request
    /// encoding and applying it requires firmware that advertises manual
    /// bandwidth control. Current RFOne firmware does not.
    pub fn bandwidth_hz(mut self, value: u32) -> Self {
        self.config = self.config.bandwidth_hz(value);
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
    /// Manual component gains use the ranges documented on [`crate::GainConfig::Manual`].
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
    pub fn open(self) -> impl MaybeFuture<Output = Result<Device<M>>> {
        let selector = self.selector;
        ready(self.config.build()).and_then(move |config| {
            let open = match selector {
                DeviceSelector::First => Either::left(HydraSdr::open()),
                DeviceSelector::Serial(serial) => Either::right(HydraSdr::open_sn(serial)),
            };
            open.and_then(move |direct| {
                direct
                    .into_device_info()
                    .map_err(|error| error.at("reading HydraSDR device metadata"))
                    .and_then(move |(direct, info)| {
                        let saved_config = config.data.clone();
                        direct
                            .into_configured(config.data)
                            .map_err(|error| error.at("applying initial HydraSDR configuration"))
                            .map(move |result| {
                                let direct = result?;
                                info.active_state.apply_config(&saved_config);
                                Ok(Device {
                                    inner: DeviceInner {
                                        direct,
                                        info,
                                        #[cfg(not(target_arch = "wasm32"))]
                                        shutdown_on_drop: true,
                                    },
                                    mode: PhantomData,
                                })
                            })
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
    pub fn decimation_mode(mut self, value: crate::DecimationMode) -> Self {
        self.config = self.config.decimation_mode(value);
        self
    }
}

/// Borrowed high-level view of one raw receive block.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SampleBlock<'a> {
    raw: &'a [u8],
    sample_count: i32,
    dropped_samples: u64,
}

impl<'a> SampleBlock<'a> {
    pub(crate) const fn new(raw: &'a [u8], sample_count: i32, dropped_samples: u64) -> Self {
        Self {
            raw,
            sample_count,
            dropped_samples,
        }
    }

    fn from_transfer(transfer: &Transfer<'a>) -> Self {
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
    pub const fn sample_count(&self) -> i32 {
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
#[cfg(not(target_arch = "wasm32"))]
enum SyncReceiverState {
    Stopped,
    StopRequired,
    Running,
}

/// Owned synchronous raw ADC stream state used by the typed stream facade.
#[cfg(not(target_arch = "wasm32"))]
struct OwnedRawRxStreamInner<C: ControlBackend + StreamingBackend> {
    device: Option<DeviceInner<C>>,
    stream: Option<DirectRawRxStream<C::BulkIn>>,
    stats: StreamingStats,
    state: SyncReceiverState,
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
            state: SyncReceiverState::Stopped,
        }
    }

    fn start(&mut self) -> Result<()> {
        if self.state == SyncReceiverState::Running {
            return Ok(());
        }
        let device = self
            .device
            .as_mut()
            .expect("owned raw stream retains its device");
        device.ensure_raw_adc_stream_format()?;
        self.state = SyncReceiverState::StopRequired;
        self.stream = Some(device.direct.start_raw_rx_stream()?);
        self.state = SyncReceiverState::Running;
        Ok(())
    }

    fn next_block(&mut self, timeout: Duration) -> Result<Option<SampleBlock<'_>>> {
        if self.state != SyncReceiverState::Running {
            return Err(Error::stream_closed("raw RX stream is stopped"));
        }
        Ok(self
            .stream
            .as_mut()
            .ok_or(Error::stream_closed("raw RX stream is closed"))?
            .next_transfer(timeout)?
            .map(|transfer| SampleBlock::from_transfer(&transfer)))
    }

    fn stop(&mut self) -> Result<StreamingStats> {
        if self.state == SyncReceiverState::Stopped {
            return Ok(self.stats);
        }
        let device = self
            .device
            .as_mut()
            .expect("owned raw stream retains its device");
        let result = if let Some(stream) = self.stream.take() {
            let (stats, result) = device.direct.close_raw_rx_stream(stream);
            self.stats = stats;
            result
        } else {
            device.direct.receiver_mode(ReceiverMode::Off).wait()
        };
        match result {
            Ok(()) => {
                self.state = SyncReceiverState::Stopped;
                Ok(self.stats)
            }
            Err(error) => {
                self.state = SyncReceiverState::StopRequired;
                Err(error)
            }
        }
    }

    fn into_device(mut self) -> DeviceInner<C> {
        let _ = self.stop();
        self.device
            .take()
            .expect("owned raw stream retains its device")
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
pub(crate) struct F32RxStreamInner<C: ControlBackend + StreamingBackend> {
    device: Option<DeviceInner<C>>,
    stream: Option<DirectRxStream<C::BulkIn>>,
    stats: StreamingStats,
    state: SyncReceiverState,
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
            state: SyncReceiverState::Stopped,
        }
    }

    fn active_state(&self) -> crate::ActiveState {
        self.device
            .as_ref()
            .expect("owned synchronous stream retains its device")
            .info
            .active_state
            .clone()
    }

    fn start(&mut self) -> Result<()> {
        if self.state == SyncReceiverState::Running {
            return Ok(());
        }
        let device = self
            .device
            .as_mut()
            .expect("owned synchronous stream retains its device");
        device.ensure_f32_iq_stream_format()?;
        self.state = SyncReceiverState::StopRequired;
        self.stream = Some(device.direct.start_rx_stream()?);
        self.state = SyncReceiverState::Running;
        Ok(())
    }

    /// Read converted complex samples into `out`.
    pub(crate) fn read(&mut self, out: &mut [Complex32], timeout: Duration) -> Result<usize> {
        if self.state != SyncReceiverState::Running {
            return Err(Error::stream_closed("synchronous F32 RX stream is stopped"));
        }
        let result = self
            .stream
            .as_mut()
            .ok_or(Error::stream_closed("F32 RX stream is closed"))?
            .read_float32_iq(out, timeout);
        if result.is_err() {
            self.state = SyncReceiverState::StopRequired;
        }
        result
    }

    /// Request receiver-off cleanup. Repeated calls are no-ops.
    pub(crate) fn stop(&mut self) -> Result<StreamingStats> {
        if self.state == SyncReceiverState::Stopped {
            return Ok(self.stats);
        }
        let device = self
            .device
            .as_mut()
            .expect("owned synchronous stream retains its device");
        let result = if let Some(stream) = self.stream.take() {
            let (stats, result) = device.direct.close_rx_stream(stream);
            self.stats = stats;
            result
        } else {
            device.direct.receiver_mode(ReceiverMode::Off).wait()
        };
        match result {
            Ok(()) => {
                self.state = SyncReceiverState::Stopped;
                Ok(self.stats)
            }
            Err(error) => {
                self.state = SyncReceiverState::StopRequired;
                Err(error)
            }
        }
    }

    fn into_device(mut self) -> DeviceInner<C> {
        let _ = self.stop();
        self.device
            .take()
            .expect("owned synchronous stream retains its device")
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
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum AsyncReceiverState {
    Stopped,
    StopRequired,
    Running,
}

pub(crate) struct AsyncRawRxStreamInner<C: ControlBackend + AsyncStreamingBackend> {
    device: Option<DeviceInner<C>>,
    stream: Option<DirectAsyncRawRxStream<C::BulkIn>>,
    stats: StreamingStats,
    state: AsyncReceiverState,
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
            state: AsyncReceiverState::Stopped,
        }
    }

    fn active_state(&self) -> crate::ActiveState {
        self.device
            .as_ref()
            .expect("owned async stream retains its device")
            .info
            .active_state
            .clone()
    }

    async fn start(&mut self) -> Result<()> {
        let stream_reusable = self
            .stream
            .as_ref()
            .is_some_and(|stream| !stream.is_closed());
        if self.state == AsyncReceiverState::Running && stream_reusable {
            return Ok(());
        }
        let device = self
            .device
            .as_mut()
            .expect("owned async stream retains its device");
        device.ensure_raw_adc_stream_format()?;
        self.state = AsyncReceiverState::StopRequired;
        if stream_reusable {
            device.direct.receiver_mode(ReceiverMode::Rx).await?;
            self.state = AsyncReceiverState::Running;
            return Ok(());
        }
        self.stream = None;
        let stream = device.direct.start_raw_rx_stream_async().await?;
        self.stream = Some(stream);
        self.state = AsyncReceiverState::Running;
        Ok(())
    }

    /// Read the next sample block.
    pub(crate) async fn next_block(&mut self) -> Result<Option<SampleBlock<'_>>> {
        if self.state != AsyncReceiverState::Running {
            return Err(Error::stream_closed("async raw RX stream is stopped"));
        }
        let stream = self
            .stream
            .as_mut()
            .ok_or(Error::stream_closed("async RX stream is closed"))?;
        Ok(stream
            .next_transfer()
            .await?
            .map(|transfer| SampleBlock::from_transfer(&transfer)))
    }

    async fn stop(&mut self) -> Result<StreamingStats> {
        if self.state == AsyncReceiverState::Stopped {
            return Ok(self.stats);
        }
        self.device
            .as_ref()
            .expect("owned async stream retains its device")
            .direct
            .receiver_mode(ReceiverMode::Off)
            .await?;
        self.state = AsyncReceiverState::Stopped;
        if let Some(stream) = self.stream.as_mut() {
            stream.pause()?;
            self.stats = stream.stats();
        }
        Ok(self.stats)
    }

    fn into_device(mut self) -> DeviceInner<C> {
        if let Some(mut stream) = self.stream.take() {
            self.stats = stream.close();
        }
        self.device
            .take()
            .expect("owned async stream retains its device")
    }
}

impl<C> Drop for AsyncRawRxStreamInner<C>
where
    C: ControlBackend + AsyncStreamingBackend,
{
    fn drop(&mut self) {
        if let Some(mut stream) = self.stream.take() {
            self.stats = stream.close();
        }
    }
}

/// Owned async stream state for converted `F32Iq` samples.
pub(crate) struct AsyncF32RxStreamInner<C: ControlBackend + AsyncStreamingBackend> {
    device: Option<DeviceInner<C>>,
    stream: Option<AsyncDirectRxStream<C::BulkIn>>,
    stats: StreamingStats,
    state: AsyncReceiverState,
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
            state: AsyncReceiverState::Stopped,
        }
    }

    fn active_state(&self) -> crate::ActiveState {
        self.device
            .as_ref()
            .expect("owned async stream retains its device")
            .info
            .active_state
            .clone()
    }

    async fn start(&mut self) -> Result<()> {
        if self.state == AsyncReceiverState::Running {
            return Ok(());
        }
        let device = self
            .device
            .as_mut()
            .expect("owned async stream retains its device");
        device.ensure_f32_iq_stream_format()?;
        if let Some(stream) = self.stream.as_mut() {
            stream.set_decimation_factor(device.direct.streaming_decimation_factor())?;
            self.state = AsyncReceiverState::StopRequired;
            device.direct.receiver_mode(ReceiverMode::Rx).await?;
            self.state = AsyncReceiverState::Running;
            return Ok(());
        }
        self.state = AsyncReceiverState::StopRequired;
        let stream = device.direct.start_rx_stream_async().await?;
        self.stream = Some(stream);
        self.state = AsyncReceiverState::Running;
        Ok(())
    }

    /// Read converted complex samples into `out`.
    pub(crate) async fn read(&mut self, out: &mut [Complex32]) -> Result<usize> {
        if self.state != AsyncReceiverState::Running {
            return Err(Error::stream_closed("async F32 RX stream is stopped"));
        }
        let result = self
            .stream
            .as_mut()
            .ok_or(Error::stream_closed("async F32 RX stream is closed"))?
            .read_float32_iq(out)
            .await;
        match result {
            Ok(written) => Ok(written),
            Err(error) => {
                let mut stream = self.stream.take().expect("stream was borrowed above");
                self.stats = stream.close();
                self.state = AsyncReceiverState::StopRequired;
                Err(error)
            }
        }
    }

    async fn stop(&mut self) -> Result<StreamingStats> {
        if self.state == AsyncReceiverState::Stopped {
            return Ok(self.stats);
        }
        self.device
            .as_ref()
            .expect("owned async stream retains its device")
            .direct
            .receiver_mode(ReceiverMode::Off)
            .await?;
        self.state = AsyncReceiverState::Stopped;
        if let Some(stream) = self.stream.as_mut() {
            stream.pause()?;
            self.stats = stream.stats();
        }
        Ok(self.stats)
    }

    fn into_device(mut self) -> DeviceInner<C> {
        if let Some(mut stream) = self.stream.take() {
            self.stats = stream.close();
        }
        self.device
            .take()
            .expect("owned async stream retains its device")
    }
}

impl<C> Drop for AsyncF32RxStreamInner<C>
where
    C: ControlBackend + AsyncStreamingBackend,
{
    fn drop(&mut self) {
        if let Some(mut stream) = self.stream.take() {
            self.stats = stream.close();
        }
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

/// Owned receive stream for sample mode `M`, defaulting to [`F32Iq`].
///
/// Create this stream with [`Device::into_rx_stream`]. Waiting the first
/// [`RxStream::start`] operation selects blocking USB on native targets;
/// awaiting it selects asynchronous USB. Do not mix waiting and awaiting on
/// one stream.
///
/// The sample mode controls which data operation exists:
///
/// ```compile_fail
/// use hydrasdr_rs::{Complex32, RawAdc, RxStream};
/// use std::time::Duration;
/// fn cannot_read_iq(stream: &mut RxStream<RawAdc>, out: &mut [Complex32]) {
///     let _ = stream.read(out, Duration::ZERO);
/// }
/// ```
///
/// ```compile_fail
/// use hydrasdr_rs::{F32Iq, RxStream};
/// use std::time::Duration;
/// fn cannot_read_raw(stream: &mut RxStream<F32Iq>) {
///     let _ = stream.next_block(Duration::ZERO);
/// }
/// ```
#[must_use = "RX streams own the device; call shutdown() for explicit hardware cleanup"]
pub struct RxStream<M: SampleMode = F32Iq> {
    state: RxStreamState,
    mode: PhantomData<fn() -> M>,
}

impl<M: SampleMode> RxStream<M> {
    fn new(device: DeviceInner<NusbControl>) -> Self {
        Self {
            state: RxStreamState::Dormant(device),
            mode: PhantomData,
        }
    }

    /// Return a shared handle to the authoritative active receiver state.
    pub fn active_state(&self) -> crate::ActiveState {
        match &self.state {
            RxStreamState::Dormant(device) => device.info.active_state.clone(),
            RxStreamState::Poisoned => panic!("RX stream state transition was interrupted"),
            #[cfg(not(target_arch = "wasm32"))]
            RxStreamState::BlockingRaw(stream) => stream
                .device
                .as_ref()
                .expect("owned raw stream retains its device")
                .info
                .active_state
                .clone(),
            #[cfg(not(target_arch = "wasm32"))]
            RxStreamState::BlockingF32(stream) => stream.active_state(),
            RxStreamState::AsyncRaw(stream) => stream.active_state(),
            RxStreamState::AsyncF32(stream) => stream.active_state(),
        }
    }

    /// Start reception and the persistent USB transfer queue.
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

    /// Consume the stopped stream and recover its typed device.
    ///
    /// Call [`RxStream::stop`] first when asynchronous receiver-off cleanup is
    /// required. Consuming a running native blocking stream stops it best-effort.
    pub fn into_device(self) -> Device<M> {
        Device {
            inner: self.into_device_inner(),
            mode: PhantomData,
        }
    }

    /// Consume the stream and explicitly turn off reception and RF bias power.
    #[must_use = "shutdown must be awaited or waited to send hardware cleanup commands"]
    pub fn shutdown(self) -> impl MaybeFuture<Output = Result<()>> {
        self.into_device_inner().shutdown()
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

    fn into_device_inner(self) -> DeviceInner<NusbControl> {
        match self.state {
            RxStreamState::Dormant(device) => device,
            RxStreamState::Poisoned => panic!("RX stream state transition was interrupted"),
            #[cfg(not(target_arch = "wasm32"))]
            RxStreamState::BlockingRaw(stream) => stream.into_device(),
            #[cfg(not(target_arch = "wasm32"))]
            RxStreamState::BlockingF32(stream) => (*stream).into_device(),
            RxStreamState::AsyncRaw(stream) => stream.into_device(),
            RxStreamState::AsyncF32(stream) => (*stream).into_device(),
        }
    }
}

impl RxStream<RawAdc> {
    /// Read the next zero-copy raw ADC USB block.
    ///
    /// The returned block borrows one buffer from the fixed transfer pool. Its
    /// buffer is resubmitted on the next call. `timeout` applies to blocking
    /// operation; asynchronous operation waits for the next USB completion.
    pub fn next_block(
        &mut self,
        timeout: Duration,
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
    /// `timeout` applies to blocking operation; asynchronous operation waits
    /// for a USB completion when no converted samples are already buffered.
    pub fn read<'a>(
        &'a mut self,
        out: &'a mut [Complex32],
        timeout: Duration,
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
    timeout: Duration,
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
        self.stream.next_block_blocking(self.timeout)
    }
}

struct ReadOperation<'a> {
    stream: &'a mut RxStream<F32Iq>,
    out: &'a mut [Complex32],
    timeout: Duration,
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
        self.stream.read_blocking(self.out, self.timeout)
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
            } else if request == VendorControlRequest::get_samplerates_count(false) {
                Ok(1u32.to_le_bytes().to_vec())
            } else if request == VendorControlRequest::get_samplerates(1, false) {
                Ok(10_000_000u32.to_le_bytes().to_vec())
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
                status: Ok(()),
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
                status: Ok(()),
            }
        }

        fn cancel_all(&mut self) {
            self.state.cancel_count.fetch_add(1, Ordering::SeqCst);
        }
    }

    fn fake_device(control: FakeControl, sample_format: SampleFormat) -> DeviceInner<FakeControl> {
        let active_state = crate::ActiveState::default();
        let config = match sample_format {
            SampleFormat::RawAdc => {
                Config::builder()
                    .raw_adc()
                    .build()
                    .expect("valid raw fake configuration")
                    .data
            }
            SampleFormat::F32Iq => Config::default().data,
        };
        active_state.apply_config(&config);
        DeviceInner {
            direct: HydraSdr::from_control(control),
            info: DeviceInfo {
                board_name: "fake HydraSDR",
                firmware_version: "fake firmware".to_owned(),
                serial: None,
                min_frequency: 24_000_000,
                max_frequency: 1_800_000_000,
                rf_ports: Vec::new(),
                active_state,
            },
            #[cfg(not(target_arch = "wasm32"))]
            shutdown_on_drop: true,
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn explicit_shutdown_turns_off_receiver_and_bias_power() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        let device = fake_device(control, SampleFormat::RawAdc);
        let active_state = device.info.active_state.clone();

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
        assert!(!active_state.bias_tee().expect("bias state after shutdown"));
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn shutdown_attempts_bias_off_after_receiver_off_fails() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        state.fail_control_out_at.store(1, Ordering::SeqCst);
        let device = fake_device(control, SampleFormat::RawAdc);
        let active_state = device.info.active_state.clone();

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
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 4);
        assert_eq!(
            *state
                .control_out_requests
                .lock()
                .expect("control request lock"),
            [
                VendorControlRequest::receiver_mode(ReceiverMode::Off),
                VendorControlRequest::set_rf_bias(0),
                VendorControlRequest::receiver_mode(ReceiverMode::Off),
                VendorControlRequest::set_rf_bias(0),
            ]
        );
        assert!(!active_state.bias_tee().expect("successful bias-off state"));
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn initial_packing_failure_does_not_enable_bias_power() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        state.fail_packing.store(true, Ordering::SeqCst);
        let config = Config::builder()
            .gain(GainConfig::Unchanged)
            .bias_tee(true)
            .build()
            .expect("valid fake configuration");

        HydraSdr::from_control(control)
            .into_configured(config.data)
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

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn dropping_unpolled_shutdown_runs_native_fallback() {
        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        let device = fake_device(control, SampleFormat::RawAdc);

        let shutdown = device.shutdown();
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 0);
        drop(shutdown);

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
    fn canceling_polled_shutdown_runs_native_fallback() {
        use std::future::IntoFuture;

        let control = FakeControl::default();
        let state = Arc::clone(&control.state);
        state.pause_control_out_at.store(1, Ordering::SeqCst);
        let device = fake_device(control, SampleFormat::RawAdc);
        let mut shutdown = Box::pin(device.shutdown().into_future());

        assert!(block_on(futures_lite::future::poll_once(shutdown.as_mut())).is_none());
        drop(shutdown);

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
        let device = fake_device(control, SampleFormat::RawAdc);

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
        let device = fake_device(control, SampleFormat::RawAdc);

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
        let device = fake_device(control, SampleFormat::F32Iq);
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

    #[test]
    fn owned_async_f32_stream_reuses_queue_and_returns_device() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control, SampleFormat::F32Iq);
            let mut stream = AsyncF32RxStreamInner::new(device);
            stream.start().await.expect("start owned async F32 stream");

            let mut first = [Complex32::default(); 1];
            let mut second = [Complex32::default(); 1];
            assert_eq!(stream.read(&mut first).await.expect("first read"), 1);
            assert_eq!(stream.read(&mut second).await.expect("second read"), 1);
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);

            let stats = stream.stop().await.expect("stop owned async F32 stream");
            let _device = stream.into_device();
            assert_eq!(stats.buffers_received, 1);
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 3);
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 2);
        });
    }

    #[test]
    fn owned_async_f32_stream_reuses_queue_across_restart() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control, SampleFormat::F32Iq);
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

            let _device = stream.into_device();
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 3);
        });
    }

    #[test]
    fn owned_async_stream_start_error_returns_device() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control, SampleFormat::RawAdc);
            let mut stream = AsyncF32RxStreamInner::new(device);
            let error = stream
                .start()
                .await
                .expect_err("F32 stream must reject raw ADC configuration");
            let device = stream.into_device();
            assert_eq!(error.kind(), crate::ErrorKind::InvalidConfig);
            assert_eq!(
                device.info.active_state.sample_format().unwrap(),
                SampleFormat::RawAdc
            );
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 0);
        });
    }

    #[test]
    fn cancelled_owned_async_raw_start_can_be_stopped() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            state.pause_control_out_at.store(2, Ordering::SeqCst);
            let device = fake_device(control, SampleFormat::RawAdc);
            let mut stream = AsyncRawRxStreamInner::new(device);

            let mut start = Box::pin(stream.start());
            assert!(futures_lite::future::poll_once(&mut start).await.is_none());
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);
            drop(start);

            stream
                .stop()
                .await
                .expect("stop cancelled owned async raw stream start");
            let _device = stream.into_device();
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 3);
        });
    }

    #[test]
    fn cancelled_owned_async_f32_start_can_be_stopped() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            state.pause_control_out_at.store(2, Ordering::SeqCst);
            let device = fake_device(control, SampleFormat::F32Iq);
            let mut stream = AsyncF32RxStreamInner::new(device);

            let mut start = Box::pin(stream.start());
            assert!(futures_lite::future::poll_once(&mut start).await.is_none());
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);
            drop(start);

            stream
                .stop()
                .await
                .expect("stop cancelled owned async F32 stream start");
            let _device = stream.into_device();
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 3);
        });
    }

    #[test]
    fn owned_async_stream_stop_error_still_returns_device() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control, SampleFormat::F32Iq);
            let mut stream = AsyncF32RxStreamInner::new(device);
            stream.start().await.expect("start owned async F32 stream");
            state.fail_control_out.store(true, Ordering::SeqCst);

            assert!(
                stream
                    .stop()
                    .await
                    .is_err_and(|error| error.kind() == crate::ErrorKind::Usb)
            );
            let device = stream.into_device();
            assert_eq!(
                device.info.active_state.sample_format().unwrap(),
                SampleFormat::F32Iq
            );
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
        });
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn native_drop_closes_async_queue_and_shuts_down_hardware() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control, SampleFormat::F32Iq);
            let mut stream = AsyncF32RxStreamInner::new(device);
            stream.start().await.expect("start owned async F32 stream");
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);

            drop(stream);
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 4);
        });
    }
}
