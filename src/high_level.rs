//! HydraSDR RFOne sync and async APIs.

use crate::config::{Config, ConfigBuilder, DeviceSelector, SampleFormat};
use crate::converter::Float32IqConverter;
use crate::device::HydraSdr;
use crate::errors::{Error, Result};
use std::time::Duration;

use crate::streaming::{
    AsyncRawRxStream as DirectAsyncRawRxStream, AsyncStreamingBackend, DirectRxStream,
    RawRxStream as DirectRawRxStream, StreamingBackend, StreamingStats, Transfer,
};
use crate::types::DeviceInfo;
use crate::usb::control::{AsyncControlBackend, ControlBackend, NusbControl};

/// High-level owned HydraSDR RFOne device handle.
///
/// Hardware-opening examples are marked `no_run`; compile-only configuration
/// examples live on [`crate::Config`] and [`crate::ConfigBuilder`].
///
/// ```no_run
/// use hydrasdr_rs::{Device, GainPreset, RfPort, SampleFormat};
///
/// fn main() -> hydrasdr_rs::Result<()> {
///     let mut dev = Device::builder()
///         .frequency_hz(100_000_000)
///         .sample_rate_hz(10_000_000)
///         .sample_format(SampleFormat::RawAdc)
///         .rf_port(RfPort::Rx0)
///         .gain(GainPreset::Linearity(12))
///         .open()?;
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
    pub fn list() -> Result<Vec<crate::DeviceDescriptor>> {
        crate::discovery::list_devices()
    }

    /// Async counterpart to [`Device::list`].
    pub async fn list_async() -> Result<Vec<crate::DeviceDescriptor>> {
        crate::discovery::list_devices_async().await
    }

    /// Start building and opening a high-level USB device.
    pub fn builder() -> DeviceBuilder {
        DeviceBuilder::default()
    }

    /// Open the first visible HydraSDR RFOne with default high-level configuration.
    pub fn open() -> Result<Self> {
        Self::builder().open()
    }

    /// Open one visible HydraSDR RFOne by serial with default high-level configuration.
    pub fn open_serial(serial: u64) -> Result<Self> {
        Self::builder().serial(serial).open()
    }

    /// Async counterpart to [`Device::open`].
    pub async fn open_async() -> Result<Self> {
        Self::builder().open_async().await
    }

    /// Async counterpart to [`Device::open_serial`].
    pub async fn open_serial_async(serial: u64) -> Result<Self> {
        Self::builder().serial(serial).open_async().await
    }

    /// Return cached device metadata.
    pub fn info(&self) -> &DeviceInfo {
        self.inner.info()
    }

    /// Refresh and return device metadata.
    pub fn refresh_info(&mut self) -> Result<&DeviceInfo> {
        self.inner.refresh_info()
    }

    /// Query supported sample rates.
    pub fn sample_rates(&mut self) -> Result<Vec<u32>> {
        self.inner.direct.get_samplerates()
    }

    /// Query supported analog bandwidths.
    pub fn bandwidths(&mut self) -> Result<Vec<u32>> {
        self.inner.direct.get_bandwidths()
    }

    /// Apply a high-level receiver configuration.
    pub fn configure(&mut self, config: &Config) -> Result<()> {
        self.inner.configure(config)
    }

    /// Apply a high-level receiver configuration through async control requests.
    pub async fn configure_async(&mut self, config: &Config) -> Result<()> {
        self.inner.configure_async(config).await
    }

    /// Refresh and return device metadata through async control requests.
    pub async fn refresh_info_async(&mut self) -> Result<&DeviceInfo> {
        self.inner.refresh_info_async().await
    }

    /// Query supported sample rates through async control requests.
    pub async fn sample_rates_async(&mut self) -> Result<Vec<u32>> {
        self.inner.direct.get_samplerates_async().await
    }

    /// Query supported analog bandwidths through async control requests.
    pub async fn bandwidths_async(&mut self) -> Result<Vec<u32>> {
        self.inner.direct.get_bandwidths_async().await
    }

    /// Start a synchronous receive stream for raw ADC USB blocks.
    pub fn raw_rx_stream(&mut self) -> Result<RawRxStream<'_>> {
        Ok(RawRxStream {
            inner: self.inner.raw_rx_stream()?,
        })
    }

    /// Start a synchronous receive stream for converted `F32Iq` samples.
    pub fn f32_rx_stream(&mut self) -> Result<F32RxStream<'_>> {
        Ok(F32RxStream {
            inner: self.inner.f32_rx_stream()?,
        })
    }

    /// Start an async receive stream for raw ADC USB blocks.
    pub async fn raw_rx_stream_async(&mut self) -> Result<AsyncRawRxStream<'_>> {
        Ok(AsyncRawRxStream {
            inner: self.inner.raw_rx_stream_async().await?,
        })
    }

    /// Start an async receive stream for converted `F32Iq` samples.
    pub async fn f32_rx_stream_async(&mut self) -> Result<AsyncF32RxStream<'_>> {
        Ok(AsyncF32RxStream {
            inner: self.inner.f32_rx_stream_async().await?,
        })
    }
}

impl<C> DeviceInner<C>
where
    C: ControlBackend,
{
    /// Wrap an already-open direct handle and query device metadata.
    pub(crate) fn from_direct(mut direct: HydraSdr<C>) -> Result<Self> {
        let info = direct.get_device_info()?;
        Ok(Self {
            direct,
            info: Some(info),
            sample_format: SampleFormat::F32Iq,
        })
    }

    /// Return cached device metadata.
    ///
    /// Handles opened through `Device` constructors always have this populated.
    pub(crate) fn info(&self) -> &DeviceInfo {
        self.info
            .as_ref()
            .expect("device info not available; call refresh_info first")
    }

    /// Refresh and return direct device metadata.
    pub(crate) fn refresh_info(&mut self) -> Result<&DeviceInfo> {
        let current_config = self
            .info
            .as_ref()
            .and_then(|info| info.current_config.clone());
        let mut info = self.direct.get_device_info()?;
        info.current_config = current_config;
        self.info = Some(info);
        Ok(self.info())
    }

    /// Apply a high-level receiver configuration through the direct layer.
    pub(crate) fn configure(&mut self, config: &Config) -> Result<()> {
        config.apply_direct(&mut self.direct)?;
        self.sample_format = config.sample_format();
        if let Some(info) = &mut self.info {
            self.direct.update_cached_device_info(info);
            info.current_config = Some(config.clone());
        }
        Ok(())
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
    C: AsyncControlBackend + ControlBackend,
{
    /// Wrap an already-open direct handle and query device metadata asynchronously.
    pub(crate) async fn from_direct_async(mut direct: HydraSdr<C>) -> Result<Self> {
        let info = direct.get_device_info_async().await?;
        Ok(Self {
            direct,
            info: Some(info),
            sample_format: SampleFormat::F32Iq,
        })
    }

    /// Apply a high-level receiver configuration through the direct async layer.
    pub(crate) async fn configure_async(&mut self, config: &Config) -> Result<()> {
        config.apply_direct_async(&mut self.direct).await?;
        self.sample_format = config.sample_format();
        if let Some(info) = &mut self.info {
            self.direct.update_cached_device_info(info);
            info.current_config = Some(config.clone());
        }
        Ok(())
    }

    /// Refresh and return direct device metadata through the async layer.
    pub(crate) async fn refresh_info_async(&mut self) -> Result<&DeviceInfo> {
        let current_config = self
            .info
            .as_ref()
            .and_then(|info| info.current_config.clone());
        let mut info = self.direct.get_device_info_async().await?;
        info.current_config = current_config;
        self.info = Some(info);
        Ok(self.info())
    }
}

impl<C> DeviceInner<C>
where
    C: AsyncControlBackend + ControlBackend + AsyncStreamingBackend,
{
    /// Start an async receive stream for raw ADC USB blocks.
    pub(crate) async fn raw_rx_stream_async(&mut self) -> Result<AsyncRawRxStreamInner<'_, C>> {
        self.ensure_raw_adc_stream_format()?;
        let stream = self.direct.start_raw_rx_stream_async().await?;
        Ok(AsyncRawRxStreamInner {
            device: self,
            stream: Some(stream),
            stats: StreamingStats::default(),
            stopped: false,
            finished: false,
        })
    }

    /// Start an async receive stream for converted `F32Iq` samples.
    pub(crate) async fn f32_rx_stream_async(&mut self) -> Result<AsyncF32RxStreamInner<'_, C>> {
        self.ensure_f32_iq_stream_format()?;
        let decimation_factor = self.direct.decimation_factor();
        let stream = self.direct.start_raw_rx_stream_async().await?;
        Ok(AsyncF32RxStreamInner {
            device: self,
            stream: Some(stream),
            converter: Float32IqConverter::default(),
            converted: Vec::new(),
            pending: Vec::new(),
            pending_start: 0,
            decimation_factor,
            stats: StreamingStats::default(),
            stopped: false,
            finished: false,
        })
    }
}

/// Builder that selects, opens, and initially configures a high-level `nusb` device.
///
/// Use [`DeviceBuilder::config`] to validate the same high-level settings without
/// opening hardware, or [`DeviceBuilder::open`] / [`DeviceBuilder::open_async`]
/// to apply them to a selected RFOne.
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

    /// Open and configure the selected device synchronously.
    pub fn open(self) -> Result<Device> {
        let selector = self.selector;
        let config = self.config.build()?;
        let direct = match selector {
            DeviceSelector::First => HydraSdr::open()?,
            DeviceSelector::Serial(serial) => HydraSdr::open_sn(serial)?,
        };
        let mut inner = DeviceInner::from_direct(direct)?;
        inner.configure(&config)?;
        Ok(Device { inner })
    }

    /// Open and configure the selected device asynchronously.
    pub async fn open_async(self) -> Result<Device> {
        let selector = self.selector;
        let config = self.config.build()?;
        let direct = match selector {
            DeviceSelector::First => HydraSdr::open_async().await?,
            DeviceSelector::Serial(serial) => HydraSdr::open_sn_async(serial).await?,
        };
        let mut inner = DeviceInner::from_direct_async(direct).await?;
        inner.configure_async(&config).await?;
        Ok(Device { inner })
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
pub(crate) struct RawRxStreamInner<'dev, C: ControlBackend + StreamingBackend> {
    device: &'dev mut DeviceInner<C>,
    stream: Option<DirectRawRxStream<C::BulkIn>>,
    stats: StreamingStats,
    stopped: bool,
    finished: bool,
}

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
pub struct RawRxStream<'dev> {
    inner: RawRxStreamInner<'dev, NusbControl>,
}

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
pub(crate) struct F32RxStreamInner<'dev, C: ControlBackend + StreamingBackend> {
    device: &'dev mut DeviceInner<C>,
    stream: Option<DirectRxStream<C::BulkIn>>,
    stats: StreamingStats,
    stopped: bool,
    finished: bool,
}

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
pub struct F32RxStream<'dev> {
    inner: F32RxStreamInner<'dev, NusbControl>,
}

impl F32RxStream<'_> {
    /// Read converted `(I, Q)` samples into `out`.
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

/// Idle async stream guard for explicit async stop/finish lifecycle control.
pub(crate) struct AsyncRawRxStreamInner<
    'dev,
    C: AsyncControlBackend + ControlBackend + AsyncStreamingBackend,
> {
    device: &'dev mut DeviceInner<C>,
    stream: Option<DirectAsyncRawRxStream<C::BulkIn>>,
    stats: StreamingStats,
    stopped: bool,
    finished: bool,
}

impl<C> AsyncRawRxStreamInner<'_, C>
where
    C: AsyncControlBackend + ControlBackend + AsyncStreamingBackend,
{
    /// Read the next sample block.
    pub(crate) async fn next_block(&mut self) -> Result<Option<SampleBlock<'_>>> {
        let sample_format = self.device.sample_format;
        let Some(stream) = self.stream.as_mut() else {
            return Err(Error::stream_closed("async RX stream is closed"));
        };
        Ok(stream
            .next_transfer()
            .await?
            .map(|transfer| SampleBlock::from_transfer(&transfer, sample_format)))
    }

    /// Request receiver-off cleanup. Repeated calls are no-ops.
    pub(crate) async fn stop(&mut self) -> Result<()> {
        if !self.stopped {
            let stream = self
                .stream
                .take()
                .ok_or(Error::stream_closed("async RX stream is closed"))?;
            self.stats = self.device.direct.stop_raw_rx_stream_async(stream).await?;
            self.stopped = true;
        }
        Ok(())
    }

    /// Finish this stream guard and return current direct streaming counters.
    pub(crate) async fn finish(mut self) -> Result<StreamingStats> {
        if !self.stopped {
            self.stop().await?;
        }
        self.finished = true;
        Ok(self.stats)
    }
}

impl<C> Drop for AsyncRawRxStreamInner<'_, C>
where
    C: AsyncControlBackend + ControlBackend + AsyncStreamingBackend,
{
    fn drop(&mut self) {
        if !self.stopped && !self.finished {
            if let Some(mut stream) = self.stream.take() {
                self.stats = stream.close();
            }
            let _ = self.device.direct.receiver_off_if_needed();
            self.stopped = true;
        }
    }
}

/// Async raw ADC block stream guard.
///
/// Call [`AsyncRawRxStream::stop`] or [`AsyncRawRxStream::finish`] to stop the
/// receiver through the async USB path. Dropping the stream cancels pending
/// transfers and attempts best-effort synchronous receiver-off cleanup.
#[must_use = "call stop().await or finish().await to stop the receiver cleanly"]
pub struct AsyncRawRxStream<'dev> {
    inner: AsyncRawRxStreamInner<'dev, NusbControl>,
}

impl AsyncRawRxStream<'_> {
    /// Read the next sample block.
    pub async fn next_block(&mut self) -> Result<Option<SampleBlock<'_>>> {
        self.inner.next_block().await
    }

    /// Request async receiver-off cleanup.
    ///
    /// Repeated calls are no-ops after the first successful stop. Prefer this
    /// over relying on drop when you are already in async code.
    pub async fn stop(&mut self) -> Result<()> {
        self.inner.stop().await
    }

    /// Stop the receiver asynchronously and return current streaming counters.
    pub async fn finish(self) -> Result<StreamingStats> {
        self.inner.finish().await
    }
}

/// Idle async converted `F32Iq` stream guard for explicit async stop/finish lifecycle control.
pub(crate) struct AsyncF32RxStreamInner<
    'dev,
    C: AsyncControlBackend + ControlBackend + AsyncStreamingBackend,
> {
    device: &'dev mut DeviceInner<C>,
    stream: Option<DirectAsyncRawRxStream<C::BulkIn>>,
    converter: Float32IqConverter,
    converted: Vec<(f32, f32)>,
    pending: Vec<(f32, f32)>,
    pending_start: usize,
    decimation_factor: usize,
    stats: StreamingStats,
    stopped: bool,
    finished: bool,
}

impl<C> AsyncF32RxStreamInner<'_, C>
where
    C: AsyncControlBackend + ControlBackend + AsyncStreamingBackend,
{
    /// Read converted `(I, Q)` samples into `out`.
    pub(crate) async fn read(&mut self, out: &mut [(f32, f32)]) -> Result<usize> {
        if out.is_empty() {
            return Ok(0);
        }

        let mut written = 0;
        self.copy_pending(out, &mut written);
        if written == out.len() {
            return Ok(written);
        }

        loop {
            let stream = self
                .stream
                .as_mut()
                .ok_or(Error::stream_closed("async F32 RX stream is closed"))?;
            let Some(transfer) = stream.next_transfer().await? else {
                return Ok(written);
            };

            self.converted.clear();
            self.converter.process_u16le_to_f32iq(
                transfer.samples,
                self.decimation_factor,
                &mut self.converted,
            );
            self.copy_converted(out, &mut written);
            if written == out.len() {
                return Ok(written);
            }
        }
    }

    /// Request receiver-off cleanup. Repeated calls are no-ops.
    pub(crate) async fn stop(&mut self) -> Result<()> {
        if !self.stopped {
            let stream = self
                .stream
                .take()
                .ok_or(Error::stream_closed("async F32 RX stream is closed"))?;
            self.stats = self.device.direct.stop_raw_rx_stream_async(stream).await?;
            self.stopped = true;
        }
        Ok(())
    }

    /// Finish this stream guard and return current streaming counters.
    pub(crate) async fn finish(mut self) -> Result<StreamingStats> {
        if !self.stopped {
            self.stop().await?;
        }
        self.finished = true;
        Ok(self.stats)
    }

    fn copy_pending(&mut self, out: &mut [(f32, f32)], written: &mut usize) {
        let pending = &self.pending[self.pending_start..];
        let take = (out.len() - *written).min(pending.len());
        if take == 0 {
            return;
        }

        out[*written..*written + take].copy_from_slice(&pending[..take]);
        self.pending_start += take;
        if self.pending_start == self.pending.len() {
            self.pending.clear();
            self.pending_start = 0;
        }
        *written += take;
    }

    fn copy_converted(&mut self, out: &mut [(f32, f32)], written: &mut usize) {
        let take = (out.len() - *written).min(self.converted.len());
        if take > 0 {
            out[*written..*written + take].copy_from_slice(&self.converted[..take]);
            *written += take;
        }
        if take < self.converted.len() {
            debug_assert_eq!(self.pending_start, 0);
            self.pending.extend_from_slice(&self.converted[take..]);
        }
    }
}

impl<C> Drop for AsyncF32RxStreamInner<'_, C>
where
    C: AsyncControlBackend + ControlBackend + AsyncStreamingBackend,
{
    fn drop(&mut self) {
        if !self.stopped && !self.finished {
            if let Some(mut stream) = self.stream.take() {
                self.stats = stream.close();
            }
            let _ = self.device.direct.receiver_off_if_needed();
            self.stopped = true;
        }
    }
}

/// Async converted `F32Iq` stream guard.
///
/// Call [`AsyncF32RxStream::stop`] or [`AsyncF32RxStream::finish`] to stop the
/// receiver through the async USB path. Dropping the stream cancels pending
/// transfers and attempts best-effort synchronous receiver-off cleanup.
#[must_use = "call stop().await or finish().await to stop the receiver cleanly"]
pub struct AsyncF32RxStream<'dev> {
    inner: AsyncF32RxStreamInner<'dev, NusbControl>,
}

impl AsyncF32RxStream<'_> {
    /// Read converted `(I, Q)` samples into `out`.
    pub async fn read(&mut self, out: &mut [(f32, f32)]) -> Result<usize> {
        self.inner.read(out).await
    }

    /// Request async receiver-off cleanup.
    ///
    /// Repeated calls are no-ops after the first successful stop. Prefer this
    /// over relying on drop when you are already in async code.
    pub async fn stop(&mut self) -> Result<()> {
        self.inner.stop().await
    }

    /// Stop the receiver asynchronously and return current streaming counters.
    pub async fn finish(self) -> Result<StreamingStats> {
        self.inner.finish().await
    }
}
