//! HydraSDR RFOne sync and async APIs.

use nusb::MaybeFuture;

use crate::commands::ReceiverMode;
use crate::config::{
    Bandwidth, Config, ConfigBuilder, DeviceSelector, GainConfig, RfPort, SampleFormat,
    validate_bandwidth, validate_frequency, validate_gain, validate_sample_rate,
};
use crate::device::HydraSdr;
use crate::errors::{Error, Result};
use crate::maybe_future::{Either, MaybeFutureExt, ready};
#[cfg(not(target_arch = "wasm32"))]
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
/// Hardware-opening examples are marked `no_run`; compile-only configuration
/// examples live on [`crate::Config`] and [`crate::ConfigBuilder`].
///
/// ```no_run
/// use hydrasdr_rs::{Device, GainPreset, MaybeFuture, RfPort, SampleFormat};
///
/// fn main() -> hydrasdr_rs::Result<()> {
///     let mut dev = Device::builder()
///         .frequency_hz(100_000_000)
///         .sample_rate_hz(10_000_000)
///         .sample_format(SampleFormat::RawAdc)
///         .rf_port(RfPort::Rx0)
///         .gain(GainPreset::Linearity(12))
///         .open()
///         .wait()?;
///
///     let mut rx = dev.raw_rx_stream()?;
///     if let Some(block) = rx.next_block()? {
///         println!("{} raw bytes", block.raw_bytes().len());
///     }
///     let stats = rx.finish()?;
///     println!("{stats:?}");
///
///     Ok(())
/// }
/// ```
#[derive(Debug)]
pub(crate) struct DeviceInner<C = NusbControl> {
    direct: HydraSdr<C>,
    info: Option<DeviceInfo>,
    sample_format: SampleFormat,
}

/// High-level owned HydraSDR RFOne device handle.
#[derive(Debug)]
pub struct Device {
    inner: DeviceInner<NusbControl>,
}

impl Device {
    /// List visible HydraSDR RFOne USB devices without opening them.
    pub fn list() -> impl MaybeFuture<Output = Result<Vec<crate::DeviceDescriptor>>> {
        crate::discovery::list_devices()
    }

    /// Start building and opening a high-level USB device.
    pub fn builder() -> DeviceBuilder {
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

    /// Return cached device metadata.
    pub fn info(&self) -> &DeviceInfo {
        self.inner.info()
    }

    /// Refresh and return device metadata.
    pub fn refresh_info(&mut self) -> impl MaybeFuture<Output = Result<&DeviceInfo>> {
        self.inner.refresh_info()
    }

    /// Query supported sample rates.
    pub fn sample_rates(&mut self) -> impl MaybeFuture<Output = Result<Vec<u32>>> {
        self.inner.direct.get_samplerates()
    }

    /// Query supported analog bandwidths.
    pub fn bandwidths(&mut self) -> impl MaybeFuture<Output = Result<Vec<u32>>> {
        self.inner.direct.get_bandwidths()
    }

    /// Apply a high-level receiver configuration.
    pub fn configure(&mut self, config: &Config) -> impl MaybeFuture<Output = Result<()>> {
        self.inner.configure(config)
    }

    /// Set only the tuned center frequency.
    pub fn set_frequency_hz(&mut self, frequency_hz: u64) -> impl MaybeFuture<Output = Result<()>> {
        self.inner.set_frequency_hz(frequency_hz)
    }

    /// Set only the sample rate.
    pub fn set_sample_rate_hz(
        &mut self,
        sample_rate_hz: u32,
    ) -> impl MaybeFuture<Output = Result<()>> {
        self.inner.set_sample_rate_hz(sample_rate_hz)
    }

    /// Set only the manual analog bandwidth.
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

    /// Start a synchronous receive stream for raw ADC USB blocks.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn raw_rx_stream(&mut self) -> Result<RawRxStream<'_>> {
        Ok(RawRxStream {
            inner: self.inner.raw_rx_stream()?,
        })
    }

    /// Start a synchronous receive stream for converted `F32Iq` samples.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn f32_rx_stream(&mut self) -> Result<F32RxStream<'_>> {
        Ok(F32RxStream {
            inner: self.inner.f32_rx_stream()?,
        })
    }

    /// Consume this device and create an owned async raw ADC receive stream.
    ///
    /// Call [`AsyncRawRxStream::start`] to start the receiver and transfer queue.
    pub fn into_async_raw_rx_stream(self) -> AsyncRawRxStream {
        AsyncRawRxStream {
            inner: AsyncRawRxStreamInner::new(self.inner),
        }
    }

    /// Consume this device and create an owned async converted `F32Iq` receive stream.
    ///
    /// Call [`AsyncF32RxStream::start`] to start the receiver and transfer queue.
    pub fn into_async_f32_rx_stream(self) -> AsyncF32RxStream {
        AsyncF32RxStream {
            inner: AsyncF32RxStreamInner::new(self.inner),
        }
    }
}

impl<C> DeviceInner<C> {
    /// Return cached device metadata.
    ///
    /// Handles opened through `Device` constructors always have this populated.
    pub(crate) fn info(&self) -> &DeviceInfo {
        self.info
            .as_ref()
            .expect("device info not available; call refresh_info first")
    }

    fn ensure_raw_adc_stream_format(&self) -> Result<()> {
        if self.sample_format != SampleFormat::RawAdc {
            return Err(Error::invalid_config(
                "sample_format",
                "raw block streams require SampleFormat::RawAdc",
            ));
        }
        Ok(())
    }

    fn ensure_f32_iq_stream_format(&self) -> Result<()> {
        if self.sample_format != SampleFormat::F32Iq {
            return Err(Error::invalid_config(
                "sample_format",
                "F32 IQ streams require SampleFormat::F32Iq",
            ));
        }
        Ok(())
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl<C> DeviceInner<C>
where
    C: ControlBackend + StreamingBackend,
{
    /// Start a synchronous receive stream for raw ADC USB blocks.
    pub(crate) fn raw_rx_stream(&mut self) -> Result<RawRxStreamInner<'_, C>> {
        self.ensure_raw_adc_stream_format()?;
        let stream = self.direct.start_raw_rx_stream()?;
        Ok(RawRxStreamInner {
            device: self,
            stream: Some(stream),
            stats: StreamingStats::default(),
            stopped: false,
            finished: false,
        })
    }

    /// Start a synchronous receive stream for converted `F32Iq` samples.
    pub(crate) fn f32_rx_stream(&mut self) -> Result<F32RxStreamInner<'_, C>> {
        self.ensure_f32_iq_stream_format()?;
        let stream = self.direct.start_rx_stream()?;
        Ok(F32RxStreamInner {
            device: self,
            stream: Some(stream),
            stats: StreamingStats::default(),
            stopped: false,
            finished: false,
        })
    }
}

impl<C> DeviceInner<C>
where
    C: ControlBackend,
{
    /// Apply a high-level receiver configuration through the direct layer.
    pub(crate) fn configure(
        &mut self,
        config: &Config,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let operation = self.direct.configure(config);
        let sample_format = &mut self.sample_format;
        let info = &mut self.info;
        let config = config.clone();
        operation.map(move |result| {
            result?;
            *sample_format = config.sample_format();
            if let Some(info) = info {
                info.current_config = Some(config);
            }
            Ok(())
        })
    }

    fn set_frequency_hz(
        &mut self,
        frequency_hz: u64,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let operation = self.direct.set_freq(frequency_hz);
        let info = &mut self.info;
        ready(validate_frequency(frequency_hz))
            .and_then(move |()| operation)
            .map(move |result| {
                result?;
                if let Some(config) = current_config_mut(info) {
                    config.update_frequency_hz(frequency_hz);
                }
                Ok(())
            })
    }

    fn set_sample_rate_hz(
        &mut self,
        sample_rate_hz: u32,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let validation = validate_sample_rate(sample_rate_hz, self.sample_format);
        let operation = self.direct.set_samplerate(sample_rate_hz);
        let info = &mut self.info;
        ready(validation)
            .and_then(move |()| operation)
            .map(move |result| {
                result?;
                if let Some(config) = current_config_mut(info) {
                    config.update_sample_rate_hz(sample_rate_hz);
                }
                Ok(())
            })
    }

    fn set_bandwidth_hz(
        &mut self,
        bandwidth_hz: u32,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let bandwidth = Bandwidth::ManualHz(bandwidth_hz);
        let operation = self.direct.set_bandwidth(bandwidth_hz);
        let info = &mut self.info;
        ready(validate_bandwidth(bandwidth))
            .and_then(move |()| operation)
            .map(move |result| {
                result?;
                if let Some(config) = current_config_mut(info) {
                    config.update_bandwidth(bandwidth);
                }
                Ok(())
            })
    }

    fn set_rf_port(&mut self, port: RfPort) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let operation = self.direct.set_rf_port(port);
        let info = &mut self.info;
        operation.map(move |result| {
            result?;
            if let Some(config) = current_config_mut(info) {
                config.update_rf_port(port);
            }
            Ok(())
        })
    }

    fn set_gain(&mut self, gain: GainConfig) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let operation = self.direct.set_gain_config(gain);
        let info = &mut self.info;
        ready(validate_gain(gain))
            .and_then(move |()| operation)
            .map(move |result| {
                result?;
                if let Some(config) = current_config_mut(info) {
                    config.update_gain(gain);
                }
                Ok(())
            })
    }

    /// Refresh and return direct device metadata.
    pub(crate) fn refresh_info(
        &mut self,
    ) -> impl MaybeFuture<Output = Result<&DeviceInfo>> + use<'_, C> {
        let current_config = self
            .info
            .as_ref()
            .and_then(|info| info.current_config.clone());
        let operation = self.direct.get_device_info();
        let info_slot = &mut self.info;
        operation.map(move |result| {
            let mut info = result?;
            info.current_config = current_config;
            *info_slot = Some(info);
            Ok(info_slot.as_ref().expect("just populated"))
        })
    }
}

fn current_config_mut(info: &mut Option<DeviceInfo>) -> Option<&mut Config> {
    info.as_mut().and_then(|info| info.current_config.as_mut())
}

/// Builder that selects, opens, and initially configures a high-level `nusb` device.
///
/// Use [`DeviceBuilder::config`] to validate the same high-level settings without
/// opening hardware, or [`DeviceBuilder::open`] to apply them to a selected RFOne.
///
/// ```
/// use hydrasdr_rs::{Device, GainPreset, SampleFormat};
///
/// let config = Device::builder()
///     .frequency_hz(433_920_000)
///     .sample_rate_hz(2_000_000)
///     .sample_format(SampleFormat::RawAdc)
///     .gain(GainPreset::Sensitivity(8))
///     .config()?;
///
/// assert_eq!(config.frequency_hz(), 433_920_000);
/// # Ok::<(), hydrasdr_rs::Error>(())
/// ```
#[derive(Clone, Debug)]
pub struct DeviceBuilder {
    selector: DeviceSelector,
    config: ConfigBuilder,
}

impl Default for DeviceBuilder {
    fn default() -> Self {
        Self {
            selector: DeviceSelector::First,
            config: Config::builder(),
        }
    }
}

impl DeviceBuilder {
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

    /// Set the ADC/sample rate in Hz.
    ///
    /// [`SampleFormat::RawAdc`] accepts `10_000..=65_535_999` Hz.
    /// [`SampleFormat::F32Iq`] accepts `10_000..=32_767_999` Hz because
    /// the hardware rate is doubled before host-side IQ conversion.
    pub fn sample_rate_hz(mut self, value: u32) -> Self {
        self.config = self.config.sample_rate_hz(value);
        self
    }

    /// Set the analog bandwidth policy.
    ///
    /// [`crate::Bandwidth::ManualHz`] values must be in the inclusive range
    /// `1_000..=65_535_999` Hz.
    pub fn bandwidth(mut self, value: crate::Bandwidth) -> Self {
        self.config = self.config.bandwidth(value);
        self
    }

    /// Set an explicit analog bandwidth in Hz.
    ///
    /// This is shorthand for [`DeviceBuilder::bandwidth`] with
    /// [`crate::Bandwidth::ManualHz`]. Values must be in the inclusive range
    /// `1_000..=65_535_999` Hz.
    pub fn bandwidth_hz(mut self, value: u32) -> Self {
        self.config = self.config.bandwidth_hz(value);
        self
    }

    /// Set the high-level sample format.
    ///
    /// The sample format determines which sample-rate range is valid.
    pub fn sample_format(mut self, value: SampleFormat) -> Self {
        self.config = self.config.sample_format(value);
        self
    }

    /// Set the firmware/host decimation policy for float IQ samples.
    ///
    /// Decimation is only valid with [`SampleFormat::F32Iq`].
    pub fn decimation_mode(mut self, value: crate::DecimationMode) -> Self {
        self.config = self.config.decimation_mode(value);
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

    /// Enable or disable packed raw-sample transfers.
    ///
    /// Packing is only valid with [`SampleFormat::RawAdc`].
    pub fn packing(mut self, enabled: bool) -> Self {
        self.config = self.config.packing(enabled);
        self
    }

    /// Build the reusable configuration represented by this builder.
    pub fn config(self) -> Result<Config> {
        self.config.build()
    }

    /// Open and configure the selected device.
    ///
    /// Await this operation in async code, or call [`MaybeFuture::wait`] on native targets.
    pub fn open(self) -> impl MaybeFuture<Output = Result<Device>> {
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
                    .and_then(move |(direct, mut info)| {
                        let sample_format = config.sample_format();
                        let saved_config = config.clone();
                        direct
                            .into_configured(config)
                            .map_err(|error| error.at("applying initial HydraSDR configuration"))
                            .map(move |result| {
                                let direct = result?;
                                info.current_config = Some(saved_config);
                                Ok(Device {
                                    inner: DeviceInner {
                                        direct,
                                        info: Some(info),
                                        sample_format,
                                    },
                                })
                            })
                    })
            })
        })
    }
}

/// Borrowed high-level view of one raw receive block.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SampleBlock<'a> {
    raw: &'a [u8],
    format: SampleFormat,
    sample_count: i32,
    dropped_samples: u64,
}

impl<'a> SampleBlock<'a> {
    pub(crate) const fn new(
        raw: &'a [u8],
        format: SampleFormat,
        sample_count: i32,
        dropped_samples: u64,
    ) -> Self {
        Self {
            raw,
            format,
            sample_count,
            dropped_samples,
        }
    }

    fn from_transfer(transfer: &Transfer<'a>, format: SampleFormat) -> Self {
        Self::new(
            transfer.samples,
            format,
            transfer.sample_count,
            transfer.dropped_samples,
        )
    }

    /// Raw USB bytes for this sample block.
    pub const fn raw_bytes(&self) -> &'a [u8] {
        self.raw
    }

    /// High-level format associated with the active receiver configuration.
    pub const fn sample_format(&self) -> SampleFormat {
        self.format
    }

    /// Sample count reported for this block.
    pub const fn sample_count(&self) -> i32 {
        self.sample_count
    }

    /// Dropped sample count reported by the streaming layer.
    pub const fn dropped_samples(&self) -> u64 {
        self.dropped_samples
    }
}

/// Idle synchronous stream guard for explicit stop/finish lifecycle control.
#[cfg(not(target_arch = "wasm32"))]
pub(crate) struct RawRxStreamInner<'dev, C: ControlBackend + StreamingBackend> {
    device: &'dev mut DeviceInner<C>,
    stream: Option<DirectRawRxStream<C::BulkIn>>,
    stats: StreamingStats,
    stopped: bool,
    finished: bool,
}

#[cfg(not(target_arch = "wasm32"))]
impl<C> RawRxStreamInner<'_, C>
where
    C: ControlBackend + StreamingBackend,
{
    /// Read the next sample block.
    pub(crate) fn next_block(&mut self) -> Result<Option<SampleBlock<'_>>> {
        let sample_format = self.device.sample_format;
        let Some(stream) = self.stream.as_mut() else {
            return Err(Error::stream_closed("RX stream is closed"));
        };
        Ok(stream
            .next_transfer()?
            .map(|transfer| SampleBlock::from_transfer(&transfer, sample_format)))
    }

    /// Request receiver-off cleanup. Repeated calls are no-ops.
    pub(crate) fn stop(&mut self) -> Result<()> {
        if !self.stopped {
            let stream = self
                .stream
                .take()
                .ok_or(Error::stream_closed("RX stream is closed"))?;
            self.stats = self.device.direct.stop_raw_rx_stream(stream)?;
            self.stopped = true;
        }
        Ok(())
    }

    /// Finish this stream guard and return current direct streaming counters.
    pub(crate) fn finish(mut self) -> Result<StreamingStats> {
        if !self.stopped {
            self.stop()?;
        }
        self.finished = true;
        Ok(self.stats)
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl<C> Drop for RawRxStreamInner<'_, C>
where
    C: ControlBackend + StreamingBackend,
{
    fn drop(&mut self) {
        if !self.stopped && !self.finished {
            if let Some(stream) = self.stream.take() {
                let _ = self.device.direct.stop_raw_rx_stream(stream);
            }
            self.stopped = true;
        }
    }
}

/// Raw ADC block stream guard for explicit stop/finish lifecycle control.
#[must_use = "RX streams keep hardware running until dropped, stopped, or finished"]
#[cfg(not(target_arch = "wasm32"))]
pub struct RawRxStream<'dev> {
    inner: RawRxStreamInner<'dev, NusbControl>,
}

#[cfg(not(target_arch = "wasm32"))]
impl RawRxStream<'_> {
    /// Read the next sample block.
    pub fn next_block(&mut self) -> Result<Option<SampleBlock<'_>>> {
        self.inner.next_block()
    }

    /// Request receiver-off cleanup. Repeated calls are no-ops.
    pub fn stop(&mut self) -> Result<()> {
        self.inner.stop()
    }

    /// Finish this stream guard and return current streaming counters.
    pub fn finish(self) -> Result<StreamingStats> {
        self.inner.finish()
    }
}

/// Idle synchronous converted `F32Iq` stream guard.
#[cfg(not(target_arch = "wasm32"))]
pub(crate) struct F32RxStreamInner<'dev, C: ControlBackend + StreamingBackend> {
    device: &'dev mut DeviceInner<C>,
    stream: Option<DirectRxStream<C::BulkIn>>,
    stats: StreamingStats,
    stopped: bool,
    finished: bool,
}

#[cfg(not(target_arch = "wasm32"))]
impl<C> F32RxStreamInner<'_, C>
where
    C: ControlBackend + StreamingBackend,
{
    /// Read converted `(I, Q)` samples into `out`.
    pub(crate) fn read(&mut self, out: &mut [(f32, f32)], timeout: Duration) -> Result<usize> {
        self.stream
            .as_mut()
            .ok_or(Error::stream_closed("F32 RX stream is closed"))?
            .read_float32_iq(out, timeout)
    }

    /// Request receiver-off cleanup. Repeated calls are no-ops.
    pub(crate) fn stop(&mut self) -> Result<()> {
        if !self.stopped {
            let stream = self
                .stream
                .take()
                .ok_or(Error::stream_closed("F32 RX stream is closed"))?;
            self.stats = self.device.direct.stop_rx_stream(stream)?;
            self.stopped = true;
        }
        Ok(())
    }

    /// Finish this stream guard and return current direct streaming counters.
    pub(crate) fn finish(mut self) -> Result<StreamingStats> {
        if !self.stopped {
            self.stop()?;
        }
        self.finished = true;
        Ok(self.stats)
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl<C> Drop for F32RxStreamInner<'_, C>
where
    C: ControlBackend + StreamingBackend,
{
    fn drop(&mut self) {
        if !self.stopped && !self.finished {
            if let Some(stream) = self.stream.take() {
                let _ = self.device.direct.stop_rx_stream(stream);
            }
            self.stopped = true;
        }
    }
}

/// Converted `F32Iq` stream guard for explicit stop/finish lifecycle control.
#[must_use = "RX streams keep hardware running until dropped, stopped, or finished"]
#[cfg(not(target_arch = "wasm32"))]
pub struct F32RxStream<'dev> {
    inner: F32RxStreamInner<'dev, NusbControl>,
}

#[cfg(not(target_arch = "wasm32"))]
impl F32RxStream<'_> {
    /// Read converted `(I, Q)` samples into `out`.
    ///
    /// `timeout` bounds the whole read call, including any additional USB
    /// completions needed to fill `out`.
    pub fn read(&mut self, out: &mut [(f32, f32)], timeout: Duration) -> Result<usize> {
        self.inner.read(out, timeout)
    }

    /// Request receiver-off cleanup. Repeated calls are no-ops.
    pub fn stop(&mut self) -> Result<()> {
        self.inner.stop()
    }

    /// Finish this stream guard and return current streaming counters.
    pub fn finish(self) -> Result<StreamingStats> {
        self.inner.finish()
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

    async fn start(&mut self) -> Result<()> {
        if self.state == AsyncReceiverState::Running {
            return Ok(());
        }
        let device = self
            .device
            .as_mut()
            .expect("owned async stream retains its device");
        device.ensure_raw_adc_stream_format()?;
        self.state = AsyncReceiverState::StopRequired;
        if self.stream.is_some() {
            device.direct.receiver_mode(ReceiverMode::Rx).await?;
            self.state = AsyncReceiverState::Running;
            return Ok(());
        }
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
        let sample_format = self
            .device
            .as_ref()
            .expect("owned async stream retains its device")
            .sample_format;
        let stream = self
            .stream
            .as_mut()
            .ok_or(Error::stream_closed("async RX stream is closed"))?;
        Ok(stream
            .next_transfer()
            .await?
            .map(|transfer| SampleBlock::from_transfer(&transfer, sample_format)))
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

/// Owned async raw ADC block stream.
///
/// This stream retains the device and one persistent USB transfer queue. Call
/// [`AsyncRawRxStream::stop`] to stop the receiver asynchronously while preserving
/// the queue for another [`AsyncRawRxStream::start`]. Consume it with
/// [`AsyncRawRxStream::into_device`] if the device handle is needed afterward.
/// Dropping a running stream closes the transfer queue and device handle, but cannot
/// perform asynchronous receiver-off cleanup. WebUSB cannot cancel pending transfers,
/// so explicit shutdown is especially important in the browser.
#[must_use = "RX streams own the device; call stop().await for receiver-off cleanup"]
pub struct AsyncRawRxStream {
    inner: AsyncRawRxStreamInner<NusbControl>,
}

impl AsyncRawRxStream {
    /// Start the receiver and persistent USB transfer queue.
    ///
    /// Repeated calls are no-ops while the stream is already running. Cancellation
    /// leaves this owned stream and its device available for another start attempt
    /// or [`AsyncRawRxStream::stop`] cleanup.
    pub async fn start(&mut self) -> Result<()> {
        self.inner.start().await
    }

    /// Read the next sample block.
    pub async fn next_block(&mut self) -> Result<Option<SampleBlock<'_>>> {
        self.inner.next_block().await
    }

    /// Stop the receiver, retain the transfer queue for restart, and return counters.
    ///
    /// Cancellation leaves this stream available so cleanup can be retried.
    pub async fn stop(&mut self) -> Result<StreamingStats> {
        self.inner.stop().await
    }

    /// Consume this stream and recover its device.
    ///
    /// Call [`AsyncRawRxStream::stop`] first to perform asynchronous receiver-off
    /// cleanup. This method still returns the device after a failed cleanup attempt.
    pub fn into_device(self) -> Device {
        Device {
            inner: self.inner.into_device(),
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

    /// Read converted `(I, Q)` samples into `out`.
    pub(crate) async fn read(&mut self, out: &mut [(f32, f32)]) -> Result<usize> {
        if self.state != AsyncReceiverState::Running {
            return Err(Error::stream_closed("async F32 RX stream is stopped"));
        }
        self.stream
            .as_mut()
            .ok_or(Error::stream_closed("async F32 RX stream is closed"))?
            .read_float32_iq(out)
            .await
    }

    fn stopped_device_mut(&mut self) -> Result<&mut DeviceInner<C>> {
        if self.state != AsyncReceiverState::Stopped {
            return Err(Error::Busy);
        }
        Ok(self
            .device
            .as_mut()
            .expect("owned async stream retains its device"))
    }

    async fn set_frequency_hz(&mut self, frequency_hz: u64) -> Result<()> {
        self.stopped_device_mut()?
            .set_frequency_hz(frequency_hz)
            .await
    }

    async fn set_sample_rate_hz(&mut self, sample_rate_hz: u32) -> Result<()> {
        self.stopped_device_mut()?
            .set_sample_rate_hz(sample_rate_hz)
            .await
    }

    async fn set_bandwidth_hz(&mut self, bandwidth_hz: u32) -> Result<()> {
        self.stopped_device_mut()?
            .set_bandwidth_hz(bandwidth_hz)
            .await
    }

    async fn set_rf_port(&mut self, port: RfPort) -> Result<()> {
        self.stopped_device_mut()?.set_rf_port(port).await
    }

    async fn set_gain(&mut self, gain: GainConfig) -> Result<()> {
        self.stopped_device_mut()?.set_gain(gain).await
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

/// Owned async converted `F32Iq` stream.
///
/// This stream retains the device and one persistent USB transfer queue. Call
/// [`AsyncF32RxStream::stop`] to stop the receiver asynchronously while preserving
/// the queue for another [`AsyncF32RxStream::start`]. Consume it with
/// [`AsyncF32RxStream::into_device`] if the device handle is needed afterward.
/// Dropping a running stream closes the transfer queue and device handle, but cannot
/// perform asynchronous receiver-off cleanup. WebUSB cannot cancel pending transfers,
/// so explicit shutdown is especially important in the browser.
#[must_use = "RX streams own the device; call stop().await for receiver-off cleanup"]
pub struct AsyncF32RxStream {
    inner: AsyncF32RxStreamInner<NusbControl>,
}

impl AsyncF32RxStream {
    /// Start the receiver and persistent USB transfer queue.
    ///
    /// Repeated calls are no-ops while the stream is already running. Cancellation
    /// leaves this owned stream and its device available for another start attempt
    /// or [`AsyncF32RxStream::stop`] cleanup.
    pub async fn start(&mut self) -> Result<()> {
        self.inner.start().await
    }

    /// Read converted `(I, Q)` samples into `out`.
    pub async fn read(&mut self, out: &mut [(f32, f32)]) -> Result<usize> {
        self.inner.read(out).await
    }

    /// Set only the tuned center frequency while the receiver is stopped.
    pub async fn set_frequency_hz(&mut self, frequency_hz: u64) -> Result<()> {
        self.inner.set_frequency_hz(frequency_hz).await
    }

    /// Set only the sample rate while the receiver is stopped.
    pub async fn set_sample_rate_hz(&mut self, sample_rate_hz: u32) -> Result<()> {
        self.inner.set_sample_rate_hz(sample_rate_hz).await
    }

    /// Set only the manual analog bandwidth while the receiver is stopped.
    pub async fn set_bandwidth_hz(&mut self, bandwidth_hz: u32) -> Result<()> {
        self.inner.set_bandwidth_hz(bandwidth_hz).await
    }

    /// Set only the RF input port while the receiver is stopped.
    pub async fn set_rf_port(&mut self, port: RfPort) -> Result<()> {
        self.inner.set_rf_port(port).await
    }

    /// Apply only the supplied gain update while the receiver is stopped.
    pub async fn set_gain(&mut self, gain: GainConfig) -> Result<()> {
        self.inner.set_gain(gain).await
    }

    /// Stop the receiver, retain the transfer queue for restart, and return counters.
    ///
    /// Cancellation leaves this stream available so cleanup can be retried.
    pub async fn stop(&mut self) -> Result<StreamingStats> {
        self.inner.stop().await
    }

    /// Consume this stream and recover its device.
    ///
    /// Call [`AsyncF32RxStream::stop`] first to perform asynchronous receiver-off
    /// cleanup. This method still returns the device after a failed cleanup attempt.
    pub fn into_device(self) -> Device {
        Device {
            inner: self.inner.into_device(),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

    use futures_lite::future::block_on;

    use super::*;
    use crate::streaming::{AsyncBulkInBackend, BulkInCompletion};
    use crate::usb::control::VendorControlRequest;

    #[derive(Debug, Default)]
    struct FakeState {
        control_out_count: AtomicUsize,
        fail_control_out: AtomicBool,
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
            let result = if request == VendorControlRequest::get_samplerates_count(false) {
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
            _request: VendorControlRequest,
        ) -> impl MaybeFuture<Output = Result<()>> + use<> {
            FakeControlOut {
                state: Arc::clone(&self.state),
            }
        }
    }

    struct FakeControlOut {
        state: Arc<FakeState>,
    }

    impl std::future::IntoFuture for FakeControlOut {
        type Output = Result<()>;
        type IntoFuture = std::pin::Pin<Box<dyn std::future::Future<Output = Result<()>> + Send>>;

        fn into_future(self) -> Self::IntoFuture {
            Box::pin(async move {
                let call = self.state.control_out_count.fetch_add(1, Ordering::SeqCst) + 1;
                if self.state.pause_control_out_at.load(Ordering::SeqCst) == call {
                    core::future::pending::<()>().await;
                }
                if self.state.fail_control_out.load(Ordering::SeqCst) {
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
            self.state.control_out_count.fetch_add(1, Ordering::SeqCst);
            if self.state.fail_control_out.load(Ordering::SeqCst) {
                Err(nusb::transfer::TransferError::Fault.into())
            } else {
                Ok(())
            }
        }
    }

    impl AsyncStreamingBackend for FakeControl {
        type BulkIn = FakeAsyncBulkIn;

        async fn bulk_in_async(&self, _endpoint: u8) -> Result<Self::BulkIn> {
            self.state.bulk_in_count.fetch_add(1, Ordering::SeqCst);
            Ok(FakeAsyncBulkIn {
                state: Arc::clone(&self.state),
                submitted: VecDeque::new(),
            })
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
            self.submitted.clear();
        }
    }

    fn fake_device(control: FakeControl, sample_format: SampleFormat) -> DeviceInner<FakeControl> {
        DeviceInner {
            direct: HydraSdr::from_control(control),
            info: None,
            sample_format,
        }
    }

    #[test]
    fn owned_async_f32_stream_reuses_queue_and_returns_device() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control, SampleFormat::F32Iq);
            let mut stream = AsyncF32RxStreamInner::new(device);
            stream.start().await.expect("start owned async F32 stream");

            let mut first = [(0.0, 0.0); 1];
            let mut second = [(0.0, 0.0); 1];
            assert_eq!(stream.read(&mut first).await.expect("first read"), 1);
            assert_eq!(stream.read(&mut second).await.expect("second read"), 1);
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);

            let stats = stream.stop().await.expect("stop owned async F32 stream");
            let _device = stream.into_device();
            assert_eq!(stats.buffers_received, 1);
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 3);
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
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
            let mut first = [(0.0, 0.0); 1];
            assert_eq!(stream.read(&mut first).await.expect("first read"), 1);
            stream.stop().await.expect("stop owned async F32 stream");
            assert_eq!(state.bulk_in_count.load(Ordering::SeqCst), 1);
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 0);
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
            let mut second = [(0.0, 0.0); 1];
            assert_eq!(stream.read(&mut second).await.expect("second read"), 1);
            stream.stop().await.expect("stop owned async F32 stream");
            assert_eq!(state.bulk_in_count.load(Ordering::SeqCst), 1);
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 0);

            let _device = stream.into_device();
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
        });
    }

    #[test]
    fn stopped_owned_async_stream_accepts_focused_updates() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control, SampleFormat::F32Iq);
            let mut stream = AsyncF32RxStreamInner::new(device);

            stream.start().await.expect("start owned async F32 stream");
            assert!(matches!(
                stream.set_frequency_hz(915_000_000).await,
                Err(Error::Busy)
            ));
            stream.stop().await.expect("stop owned async F32 stream");

            stream
                .set_frequency_hz(915_000_000)
                .await
                .expect("set focused frequency while stopped");
            stream
                .set_sample_rate_hz(2_500_000)
                .await
                .expect("set focused sample rate while stopped");
            assert_eq!(
                stream
                    .stream
                    .as_ref()
                    .expect("persistent stream")
                    .decimation_factor(),
                1
            );
            assert_eq!(state.bulk_in_count.load(Ordering::SeqCst), 1);
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 0);
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 4);

            stream
                .start()
                .await
                .expect("restart updated owned async F32 stream");
            assert_eq!(
                stream
                    .stream
                    .as_ref()
                    .expect("persistent stream")
                    .decimation_factor(),
                4
            );
            stream.stop().await.expect("stop owned async F32 stream");
            let _device = stream.into_device();
            assert_eq!(state.bulk_in_count.load(Ordering::SeqCst), 1);
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
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
            assert_eq!(device.sample_format, SampleFormat::RawAdc);
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
            assert_eq!(device.sample_format, SampleFormat::F32Iq);
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
        });
    }

    #[test]
    fn dropping_owned_async_stream_closes_queue_without_sync_control() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control, SampleFormat::F32Iq);
            let mut stream = AsyncF32RxStreamInner::new(device);
            stream.start().await.expect("start owned async F32 stream");
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);

            drop(stream);
            assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);
        });
    }
}
