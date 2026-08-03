//! HydraSDR RFOne sync and async APIs.

use crate::commands::ReceiverMode;
use crate::config::{Config, ConfigBuilder, DeviceSelector, SampleFormat};
use crate::device::HydraSdr;
use crate::errors::{Error, Result};
#[cfg(not(target_arch = "wasm32"))]
use std::time::Duration;

use crate::streaming::{
    AsyncDirectRxStream, AsyncRawRxStream as DirectAsyncRawRxStream, AsyncStreamingBackend,
    StreamingStats, Transfer,
};
#[cfg(not(target_arch = "wasm32"))]
use crate::streaming::{DirectRxStream, RawRxStream as DirectRawRxStream, StreamingBackend};
use crate::types::DeviceInfo;
#[cfg(not(target_arch = "wasm32"))]
use crate::usb::control::ControlBackend;
use crate::usb::control::{AsyncControlBackend, NusbControl};

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
    #[cfg(not(target_arch = "wasm32"))]
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
    #[cfg(not(target_arch = "wasm32"))]
    pub fn open() -> Result<Self> {
        Self::builder().open()
    }

    /// Open one visible HydraSDR RFOne by serial with default high-level configuration.
    #[cfg(not(target_arch = "wasm32"))]
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
    #[cfg(not(target_arch = "wasm32"))]
    pub fn refresh_info(&mut self) -> Result<&DeviceInfo> {
        self.inner.refresh_info()
    }

    /// Query supported sample rates.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn sample_rates(&mut self) -> Result<Vec<u32>> {
        self.inner.direct.get_samplerates()
    }

    /// Query supported analog bandwidths.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn bandwidths(&mut self) -> Result<Vec<u32>> {
        self.inner.direct.get_bandwidths()
    }

    /// Apply a high-level receiver configuration.
    #[cfg(not(target_arch = "wasm32"))]
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
    C: AsyncControlBackend,
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
    #[cfg(not(target_arch = "wasm32"))]
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
pub(crate) struct AsyncRawRxStreamInner<C: AsyncControlBackend + AsyncStreamingBackend> {
    device: Option<DeviceInner<C>>,
    stream: Option<DirectAsyncRawRxStream<C::BulkIn>>,
    stats: StreamingStats,
    receiver_needs_stop: bool,
}

impl<C> AsyncRawRxStreamInner<C>
where
    C: AsyncControlBackend + AsyncStreamingBackend,
{
    fn new(device: DeviceInner<C>) -> Self {
        Self {
            device: Some(device),
            stream: None,
            stats: StreamingStats::default(),
            receiver_needs_stop: false,
        }
    }

    async fn start(&mut self) -> Result<()> {
        if self.stream.is_some() {
            return Ok(());
        }
        let device = self
            .device
            .as_mut()
            .expect("owned async stream retains its device");
        device.ensure_raw_adc_stream_format()?;
        self.receiver_needs_stop = true;
        let stream = device.direct.start_raw_rx_stream_async().await?;
        self.stream = Some(stream);
        Ok(())
    }

    /// Read the next sample block.
    pub(crate) async fn next_block(&mut self) -> Result<Option<SampleBlock<'_>>> {
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

    async fn finish(&mut self) -> Result<StreamingStats> {
        if self.receiver_needs_stop {
            self.device
                .as_ref()
                .expect("owned async stream retains its device")
                .direct
                .receiver_mode_async(ReceiverMode::Off)
                .await?;
            self.receiver_needs_stop = false;
        }
        if let Some(mut stream) = self.stream.take() {
            self.stats = stream.close();
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
    C: AsyncControlBackend + AsyncStreamingBackend,
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
/// [`AsyncRawRxStream::finish`] to stop the receiver asynchronously, then
/// [`AsyncRawRxStream::into_device`] to recover the device. Dropping an unfinished
/// stream closes the transfer queue and device handle, but cannot perform asynchronous
/// receiver-off cleanup. WebUSB cannot cancel pending transfers, so explicit shutdown
/// is especially important in the browser.
#[must_use = "call finish().await, then into_device(), to stop RX and recover the device"]
pub struct AsyncRawRxStream {
    inner: AsyncRawRxStreamInner<NusbControl>,
}

impl AsyncRawRxStream {
    /// Start the receiver and persistent USB transfer queue.
    ///
    /// Repeated calls are no-ops while the stream is already running. Cancellation
    /// leaves this owned stream and its device available for another start attempt
    /// or [`AsyncRawRxStream::finish`] cleanup.
    pub async fn start(&mut self) -> Result<()> {
        self.inner.start().await
    }

    /// Read the next sample block.
    pub async fn next_block(&mut self) -> Result<Option<SampleBlock<'_>>> {
        self.inner.next_block().await
    }

    /// Close the transfer queue, stop the receiver, and return streaming counters.
    ///
    /// Cancellation leaves this stream available so cleanup can be retried.
    pub async fn finish(&mut self) -> Result<StreamingStats> {
        self.inner.finish().await
    }

    /// Consume this stream and recover its device.
    ///
    /// Call [`AsyncRawRxStream::finish`] first to perform asynchronous receiver-off
    /// cleanup. This method still returns the device after a failed cleanup attempt.
    pub fn into_device(self) -> Device {
        Device {
            inner: self.inner.into_device(),
        }
    }
}

/// Owned async stream state for converted `F32Iq` samples.
pub(crate) struct AsyncF32RxStreamInner<C: AsyncControlBackend + AsyncStreamingBackend> {
    device: Option<DeviceInner<C>>,
    stream: Option<AsyncDirectRxStream<C::BulkIn>>,
    stats: StreamingStats,
    receiver_needs_stop: bool,
}

impl<C> AsyncF32RxStreamInner<C>
where
    C: AsyncControlBackend + AsyncStreamingBackend,
{
    fn new(device: DeviceInner<C>) -> Self {
        Self {
            device: Some(device),
            stream: None,
            stats: StreamingStats::default(),
            receiver_needs_stop: false,
        }
    }

    async fn start(&mut self) -> Result<()> {
        if self.stream.is_some() {
            return Ok(());
        }
        let device = self
            .device
            .as_mut()
            .expect("owned async stream retains its device");
        device.ensure_f32_iq_stream_format()?;
        self.receiver_needs_stop = true;
        let stream = device.direct.start_rx_stream_async().await?;
        self.stream = Some(stream);
        Ok(())
    }

    /// Read converted `(I, Q)` samples into `out`.
    pub(crate) async fn read(&mut self, out: &mut [(f32, f32)]) -> Result<usize> {
        self.stream
            .as_mut()
            .ok_or(Error::stream_closed("async F32 RX stream is closed"))?
            .read_float32_iq(out)
            .await
    }

    async fn finish(&mut self) -> Result<StreamingStats> {
        if self.receiver_needs_stop {
            self.device
                .as_ref()
                .expect("owned async stream retains its device")
                .direct
                .receiver_mode_async(ReceiverMode::Off)
                .await?;
            self.receiver_needs_stop = false;
        }
        if let Some(mut stream) = self.stream.take() {
            self.stats = stream.close();
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
    C: AsyncControlBackend + AsyncStreamingBackend,
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
/// [`AsyncF32RxStream::finish`] to stop the receiver asynchronously, then
/// [`AsyncF32RxStream::into_device`] to recover the device. Dropping an unfinished
/// stream closes the transfer queue and device handle, but cannot perform asynchronous
/// receiver-off cleanup. WebUSB cannot cancel pending transfers, so explicit shutdown
/// is especially important in the browser.
#[must_use = "call finish().await, then into_device(), to stop RX and recover the device"]
pub struct AsyncF32RxStream {
    inner: AsyncF32RxStreamInner<NusbControl>,
}

impl AsyncF32RxStream {
    /// Start the receiver and persistent USB transfer queue.
    ///
    /// Repeated calls are no-ops while the stream is already running. Cancellation
    /// leaves this owned stream and its device available for another start attempt
    /// or [`AsyncF32RxStream::finish`] cleanup.
    pub async fn start(&mut self) -> Result<()> {
        self.inner.start().await
    }

    /// Read converted `(I, Q)` samples into `out`.
    pub async fn read(&mut self, out: &mut [(f32, f32)]) -> Result<usize> {
        self.inner.read(out).await
    }

    /// Close the transfer queue, stop the receiver, and return streaming counters.
    ///
    /// Cancellation leaves this stream available so cleanup can be retried.
    pub async fn finish(&mut self) -> Result<StreamingStats> {
        self.inner.finish().await
    }

    /// Consume this stream and recover its device.
    ///
    /// Call [`AsyncF32RxStream::finish`] first to perform asynchronous receiver-off
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
        cancel_count: AtomicUsize,
    }

    #[derive(Clone, Debug, Default)]
    struct FakeControl {
        state: Arc<FakeState>,
    }

    impl AsyncControlBackend for FakeControl {
        async fn control_in_async(&self, _request: VendorControlRequest) -> Result<Vec<u8>> {
            Ok(Vec::new())
        }

        async fn control_out_async(&self, _request: VendorControlRequest) -> Result<()> {
            let call = self.state.control_out_count.fetch_add(1, Ordering::SeqCst) + 1;
            if self.state.pause_control_out_at.load(Ordering::SeqCst) == call {
                core::future::pending::<()>().await;
            }
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

            let stats = stream
                .finish()
                .await
                .expect("finish owned async F32 stream");
            let _device = stream.into_device();
            assert_eq!(stats.buffers_received, 1);
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 3);
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
    fn cancelled_owned_async_raw_start_can_be_finished() {
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
                .finish()
                .await
                .expect("finish cancelled owned async raw stream start");
            let _device = stream.into_device();
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 3);
        });
    }

    #[test]
    fn cancelled_owned_async_f32_start_can_be_finished() {
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
                .finish()
                .await
                .expect("finish cancelled owned async F32 stream start");
            let _device = stream.into_device();
            assert_eq!(state.control_out_count.load(Ordering::SeqCst), 3);
        });
    }

    #[test]
    fn owned_async_stream_finish_error_still_returns_device() {
        block_on(async {
            let control = FakeControl::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control, SampleFormat::F32Iq);
            let mut stream = AsyncF32RxStreamInner::new(device);
            stream.start().await.expect("start owned async F32 stream");
            state.fail_control_out.store(true, Ordering::SeqCst);

            assert!(
                stream
                    .finish()
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
