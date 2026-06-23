//! Ergonomic HydraSDR RFOne API.

use crate::config::{Config, ConfigBuilder, DeviceSelector, SampleFormat};
use crate::device::HydraSdr;
use crate::errors::{Error, Result};
use crate::streaming::{AsyncStreamingBackend, StreamingBackend, StreamingStats, Transfer};
use crate::types::DeviceInfo;
use crate::usb::control::{AsyncControlBackend, ControlBackend, NusbControl};

/// High-level owned HydraSDR RFOne device handle.
///
/// Hardware-opening examples are marked `no_run`; compile-only configuration
/// examples live on [`crate::Config`] and [`crate::ConfigBuilder`].
///
/// ```no_run
/// use hydrasdr_rs::{Device, GainPreset, RfPort, SampleBlock, SampleFormat};
///
/// fn main() -> hydrasdr_rs::Result<()> {
///     let mut dev = Device::builder()
///         .frequency_hz(100_000_000)
///         .sample_rate_hz(10_000_000)
///         .sample_format(SampleFormat::RawU8Iq)
///         .rf_port(RfPort::Rx0)
///         .gain(GainPreset::Linearity(12))
///         .open()?;
///
///     let stats = dev.receive_blocks(|block: SampleBlock<'_>| {
///         println!("{} raw bytes", block.raw_bytes().len());
///         true
///     })?;
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

    /// Create an idle synchronous stream guard.
    pub fn rx_stream(&mut self) -> Result<RxStream<'_>> {
        Ok(RxStream {
            inner: self.inner.rx_stream()?,
        })
    }

    /// Create an idle async stream guard.
    pub async fn rx_stream_async(&mut self) -> Result<AsyncRxStream<'_>> {
        Ok(AsyncRxStream {
            inner: self.inner.rx_stream_async().await?,
        })
    }

    /// Receive sample blocks through the high-level callback streaming loop.
    ///
    /// The callback returns `true` to stop the stream and `false` to continue.
    pub fn receive_blocks<F>(&mut self, callback: F) -> Result<StreamingStats>
    where
        F: FnMut(SampleBlock<'_>) -> bool,
    {
        self.inner.receive_blocks(callback)
    }

    /// Async counterpart to [`Device::receive_blocks`].
    pub async fn receive_blocks_async<F>(&mut self, callback: F) -> Result<StreamingStats>
    where
        F: FnMut(SampleBlock<'_>) -> bool,
    {
        self.inner.receive_blocks_async(callback).await
    }
}

impl<C> DeviceInner<C>
where
    C: ControlBackend,
{
    /// Wrap an already-open direct handle and query device metadata.
    pub fn from_direct(mut direct: HydraSdr<C>) -> Result<Self> {
        let info = direct.get_device_info()?;
        Ok(Self {
            direct,
            info: Some(info),
            sample_format: SampleFormat::F32Iq,
        })
    }

    /// Wrap an already-open direct handle without querying hardware metadata.
    ///
    /// This is mainly useful for no-hardware tests and low-level integration
    /// harnesses that provide fake direct backends.
    pub fn from_direct_without_info(direct: HydraSdr<C>) -> Self {
        Self {
            direct,
            info: None,
            sample_format: SampleFormat::F32Iq,
        }
    }

    /// Return cached device metadata.
    ///
    /// Handles opened through [`Device::open`], [`Device::open_serial`], or
    /// [`DeviceInner::from_direct`] always have this populated. Test/fake handles
    /// created with [`DeviceInner::from_direct_without_info`] can call
    /// [`Device::refresh_info`] first if metadata is needed.
    pub fn info(&self) -> &DeviceInfo {
        self.info
            .as_ref()
            .expect("device info not available; call refresh_info first")
    }

    /// Refresh and return direct device metadata.
    pub fn refresh_info(&mut self) -> Result<&DeviceInfo> {
        self.info = Some(self.direct.get_device_info()?);
        Ok(self.info())
    }

    /// Apply a high-level receiver configuration through the direct layer.
    pub fn configure(&mut self, config: &Config) -> Result<()> {
        config.apply_direct(&mut self.direct)?;
        self.sample_format = config.sample_format();
        Ok(())
    }

    /// Borrow the underlying direct C-style handle.
    pub const fn direct(&self) -> &HydraSdr<C> {
        &self.direct
    }

    /// Create an idle synchronous stream guard.
    pub fn rx_stream(&mut self) -> Result<RxStreamInner<'_, C>> {
        if self.direct.is_streaming() {
            return Err(Error::stream_closed("direct receiver is already streaming"));
        }
        Ok(RxStreamInner {
            device: self,
            stopped: false,
            finished: false,
        })
    }
}

impl<C> DeviceInner<C>
where
    C: ControlBackend + StreamingBackend,
{
    /// Receive sample blocks through the direct callback streaming loop.
    ///
    /// The callback returns `true` to stop the stream and `false` to continue.
    ///
    /// ```no_run
    /// use hydrasdr_rs::{Device, SampleBlock};
    ///
    /// fn main() -> hydrasdr_rs::Result<()> {
    ///     let mut dev = Device::open()?;
    ///     let stats = dev.receive_blocks(|block: SampleBlock<'_>| {
    ///         println!("{} samples", block.sample_count());
    ///         true
    ///     })?;
    ///     println!("{stats:?}");
    ///     Ok(())
    /// }
    /// ```
    pub fn receive_blocks<F>(&mut self, mut callback: F) -> Result<StreamingStats>
    where
        F: FnMut(SampleBlock<'_>) -> bool,
    {
        let sample_format = self.sample_format;
        self.direct.start_rx(|transfer: &Transfer<'_>| {
            let block = SampleBlock::from_transfer(transfer, sample_format);
            i32::from(callback(block))
        })
    }
}

impl<C> DeviceInner<C>
where
    C: AsyncControlBackend + ControlBackend,
{
    /// Apply a high-level receiver configuration through the direct async layer.
    pub async fn configure_async(&mut self, config: &Config) -> Result<()> {
        config.apply_direct_async(&mut self.direct).await?;
        self.sample_format = config.sample_format();
        Ok(())
    }

    /// Refresh and return direct device metadata through the async layer.
    pub async fn refresh_info_async(&mut self) -> Result<&DeviceInfo> {
        self.info = Some(self.direct.get_device_info_async().await?);
        Ok(self.info())
    }

    /// Create an idle async stream guard.
    pub async fn rx_stream_async(&mut self) -> Result<AsyncRxStreamInner<'_, C>> {
        if self.direct.is_streaming() {
            return Err(Error::stream_closed("direct receiver is already streaming"));
        }
        Ok(AsyncRxStreamInner {
            device: self,
            stopped: false,
            finished: false,
        })
    }
}

impl<C> DeviceInner<C>
where
    C: AsyncControlBackend + ControlBackend + AsyncStreamingBackend,
{
    /// Async counterpart to [`Device::receive_blocks`].
    pub async fn receive_blocks_async<F>(&mut self, mut callback: F) -> Result<StreamingStats>
    where
        F: FnMut(SampleBlock<'_>) -> bool,
    {
        let sample_format = self.sample_format;
        self.direct
            .start_rx_async(|transfer: &Transfer<'_>| {
                let block = SampleBlock::from_transfer(transfer, sample_format);
                i32::from(callback(block))
            })
            .await
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
///     .sample_format(SampleFormat::RawU8Iq)
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
    /// Return the current device selector.
    pub const fn selector(&self) -> DeviceSelector {
        self.selector
    }

    /// Select a device by parsed 64-bit serial number.
    pub fn serial(mut self, serial: u64) -> Self {
        self.selector = DeviceSelector::Serial(serial);
        self
    }

    pub fn frequency_hz(mut self, value: u64) -> Self {
        self.config = self.config.frequency_hz(value);
        self
    }

    pub fn sample_rate_hz(mut self, value: u32) -> Self {
        self.config = self.config.sample_rate_hz(value);
        self
    }

    pub fn bandwidth(mut self, value: crate::Bandwidth) -> Self {
        self.config = self.config.bandwidth(value);
        self
    }

    pub fn bandwidth_hz(mut self, value: u32) -> Self {
        self.config = self.config.bandwidth_hz(value);
        self
    }

    pub fn sample_format(mut self, value: SampleFormat) -> Self {
        self.config = self.config.sample_format(value);
        self
    }

    pub fn rf_port(mut self, value: crate::RfPort) -> Self {
        self.config = self.config.rf_port(value);
        self
    }

    pub fn gain(mut self, value: impl Into<crate::GainConfig>) -> Self {
        self.config = self.config.gain(value);
        self
    }

    pub fn bias_tee(mut self, enabled: bool) -> Self {
        self.config = self.config.bias_tee(enabled);
        self
    }

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
        let mut inner = DeviceInner::from_direct(direct)?;
        inner.configure_async(&config).await?;
        Ok(Device { inner })
    }
}

/// Borrowed high-level view of one receive callback block.
///
/// `SampleBlock::new` is available for tests and adapters that already have raw
/// bytes and direct streaming metadata:
///
/// ```
/// use hydrasdr_rs::{SampleBlock, SampleFormat};
///
/// let raw = [0_u8, 127, 255, 128];
/// let block = SampleBlock::new(&raw, SampleFormat::RawU8Iq, 2, 0);
///
/// assert_eq!(block.raw_bytes(), &raw);
/// assert_eq!(block.sample_format(), SampleFormat::RawU8Iq);
/// assert_eq!(block.sample_count(), 2);
/// assert_eq!(block.dropped_samples(), 0);
/// ```
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SampleBlock<'a> {
    raw: &'a [u8],
    format: SampleFormat,
    sample_count: i32,
    dropped_samples: u64,
}

impl<'a> SampleBlock<'a> {
    /// Build a sample block from raw bytes and metadata.
    pub const fn new(
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

    fn from_transfer(transfer: &'a Transfer<'a>, format: SampleFormat) -> Self {
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

    /// C-parity sample count for this block.
    pub const fn sample_count(&self) -> i32 {
        self.sample_count
    }

    /// C-parity dropped sample count reported by the direct streaming layer.
    pub const fn dropped_samples(&self) -> u64 {
        self.dropped_samples
    }
}

/// Idle synchronous stream guard for explicit stop/finish lifecycle control.
pub(crate) struct RxStreamInner<'dev, C: ControlBackend> {
    device: &'dev mut DeviceInner<C>,
    stopped: bool,
    finished: bool,
}

impl<C> RxStreamInner<'_, C>
where
    C: ControlBackend,
{
    /// Request receiver-off cleanup. Repeated calls are no-ops.
    pub fn stop(&mut self) -> Result<()> {
        if !self.stopped {
            self.device.direct.stop_rx()?;
            self.stopped = true;
        }
        Ok(())
    }

    /// Finish this stream guard and return current direct streaming counters.
    pub fn finish(mut self) -> Result<StreamingStats> {
        if !self.stopped {
            self.stop()?;
        }
        self.finished = true;
        Ok(self.device.direct.streaming_stats())
    }
}

impl<C: ControlBackend> Drop for RxStreamInner<'_, C> {
    fn drop(&mut self) {
        if !self.stopped && !self.finished {
            let _ = self.device.direct.stop_rx();
            self.stopped = true;
        }
    }
}

/// Idle synchronous stream guard for explicit stop/finish lifecycle control.
pub struct RxStream<'dev> {
    inner: RxStreamInner<'dev, NusbControl>,
}

impl RxStream<'_> {
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
pub(crate) struct AsyncRxStreamInner<'dev, C: AsyncControlBackend + ControlBackend> {
    device: &'dev mut DeviceInner<C>,
    stopped: bool,
    finished: bool,
}

impl<C> AsyncRxStreamInner<'_, C>
where
    C: AsyncControlBackend + ControlBackend,
{
    /// Request receiver-off cleanup. Repeated calls are no-ops.
    pub async fn stop(&mut self) -> Result<()> {
        if !self.stopped {
            self.device.direct.stop_rx_async().await?;
            self.stopped = true;
        }
        Ok(())
    }

    /// Finish this stream guard and return current direct streaming counters.
    pub async fn finish(mut self) -> Result<StreamingStats> {
        if !self.stopped {
            self.stop().await?;
        }
        self.finished = true;
        Ok(self.device.direct.streaming_stats())
    }
}

/// Idle async stream guard for explicit async stop/finish lifecycle control.
pub struct AsyncRxStream<'dev> {
    inner: AsyncRxStreamInner<'dev, NusbControl>,
}

impl AsyncRxStream<'_> {
    /// Request receiver-off cleanup. Repeated calls are no-ops.
    pub async fn stop(&mut self) -> Result<()> {
        self.inner.stop().await
    }

    /// Finish this stream guard and return current streaming counters.
    pub async fn finish(self) -> Result<StreamingStats> {
        self.inner.finish().await
    }
}
