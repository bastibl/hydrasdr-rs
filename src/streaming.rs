//! Direct C-style streaming state machine and backend traits.

use std::future::Future;
use std::ops::Deref;
use std::time::Duration;

use crate::constants::{DEFAULT_BUFFER_SIZE, PACKED_BUFFER_SIZE};
use crate::converter::Float32IqConverter;
use crate::errors::{Error, Result, StatusCode};
use crate::rfone::RFONE_TRANSFER_COUNT;
use crate::types::SampleType;

/// Transfer view passed to a receive callback.
///
/// The buffer contains raw USB bytes. `sample_count` follows the C driver's byte-count formula,
/// not a final typed IQ sample abstraction.
#[derive(Debug)]
pub struct Transfer<'a> {
    pub samples: &'a [u8],
    pub sample_count: i32,
    pub dropped_samples: u64,
    pub sample_type: SampleType,
}

/// C-style sample-block callback. Returning non-zero requests stream stop.
pub type SampleBlockCallback = dyn FnMut(&Transfer<'_>) -> i32 + Send;

/// Counters collected during a direct streaming run.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct StreamingStats {
    pub buffers_received: u64,
    pub buffers_processed: u64,
    pub buffers_dropped: u64,
}

/// Completed bulk-IN transfer from a backend.
#[derive(Debug)]
pub struct BulkInCompletion<B> {
    pub buffer: B,
    pub actual_len: usize,
    pub status: Result<()>,
}

/// Minimal synchronous bulk-IN backend used by direct streaming tests and `nusb`.
pub trait BulkInBackend: std::fmt::Debug {
    type Buffer: Deref<Target = [u8]>;

    fn clear_halt(&mut self) -> Result<()>;
    fn allocate(&self, len: usize) -> Self::Buffer;
    fn submit(&mut self, buffer: Self::Buffer);
    fn pending(&self) -> usize;
    fn wait_next_complete(&mut self, timeout: Duration) -> Option<BulkInCompletion<Self::Buffer>>;
    fn cancel_all(&mut self);
}

/// Provider of synchronous bulk-IN endpoints.
pub trait StreamingBackend: std::fmt::Debug {
    type BulkIn: BulkInBackend;

    fn bulk_in(&self, endpoint: u8) -> Result<Self::BulkIn>;
}

/// Minimal async bulk-IN backend used by direct async streaming.
pub trait AsyncBulkInBackend: std::fmt::Debug {
    type Buffer: Deref<Target = [u8]>;

    fn clear_halt_async(&mut self) -> impl Future<Output = Result<()>> + '_;
    fn allocate(&self, len: usize) -> Self::Buffer;
    fn submit(&mut self, buffer: Self::Buffer);
    fn pending(&self) -> usize;
    fn next_complete_async(&mut self) -> impl Future<Output = BulkInCompletion<Self::Buffer>> + '_;
    fn cancel_all(&mut self);
}

/// Provider of async bulk-IN endpoints.
pub trait AsyncStreamingBackend: std::fmt::Debug {
    type BulkIn: AsyncBulkInBackend;

    fn bulk_in_async(&self, endpoint: u8) -> impl Future<Output = Result<Self::BulkIn>> + '_;
}

/// C-parity streaming buffer configuration.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct StreamingConfig {
    pub transfer_count: usize,
    pub buffer_size: usize,
    pub packed_buffer_size: usize,
    pub packing_enabled: bool,
    pub decimation_factor: usize,
    pub transfer_timeout: Duration,
}

impl Default for StreamingConfig {
    fn default() -> Self {
        Self {
            transfer_count: RFONE_TRANSFER_COUNT as usize,
            buffer_size: DEFAULT_BUFFER_SIZE,
            packed_buffer_size: PACKED_BUFFER_SIZE,
            packing_enabled: false,
            decimation_factor: 1,
            transfer_timeout: Duration::MAX,
        }
    }
}

/// Mutable state for one direct RX streaming loop.
#[derive(Debug, Default)]
pub struct StreamingState {
    config: StreamingConfig,
    streaming: bool,
    stop_requested: bool,
    stats: StreamingStats,
}

impl StreamingState {
    /// Create an idle streaming state with C RFOne defaults.
    pub fn new() -> Self {
        Self::default()
    }

    /// Return counters from the last/current run.
    pub fn stats(&self) -> StreamingStats {
        self.stats
    }

    /// Return the current streaming configuration.
    pub fn config(&self) -> StreamingConfig {
        self.config
    }

    /// Report whether the direct streaming loop is active.
    pub fn is_streaming(&self) -> bool {
        self.streaming
    }

    /// Request that the current streaming loop stop at the next safe point.
    pub fn request_stop(&mut self) {
        self.stop_requested = true;
    }

    /// Enable or disable packed samples before streaming starts.
    pub fn set_packing(&mut self, enabled: bool) -> Result<()> {
        if self.streaming {
            return Err(Error::Status(StatusCode::Busy));
        }
        self.config.packing_enabled = enabled;
        Ok(())
    }

    /// Set the DDC decimation factor before streaming starts.
    pub fn set_decimation(&mut self, factor: usize) -> Result<()> {
        if self.streaming {
            return Err(Error::Status(StatusCode::Busy));
        }
        if !matches!(factor, 1 | 2 | 4 | 8 | 16 | 32 | 64) {
            return Err(Error::Status(StatusCode::InvalidParam));
        }
        self.config.decimation_factor = factor;
        Ok(())
    }

    /// Current USB buffer size, accounting for packed mode.
    pub fn current_buffer_size(&self) -> usize {
        if self.config.packing_enabled {
            self.config.packed_buffer_size
        } else {
            self.config.buffer_size
        }
    }

    /// Run the synchronous C-style receive loop on an already-open bulk endpoint.
    pub fn run<B, F>(
        &mut self,
        mut bulk_in: B,
        sample_type: SampleType,
        mut callback: F,
    ) -> Result<StreamingStats>
    where
        B: BulkInBackend,
        F: FnMut(&Transfer<'_>) -> i32,
    {
        if self.streaming {
            return Err(Error::Status(StatusCode::Busy));
        }

        self.stats = StreamingStats::default();
        self.stop_requested = false;
        self.streaming = true;

        let result = self.run_inner(&mut bulk_in, sample_type, &mut callback);
        bulk_in.cancel_all();
        self.streaming = false;
        self.stop_requested = false;

        result.map(|()| self.stats)
    }

    /// Run the async C-style receive loop on an already-open bulk endpoint.
    pub async fn run_async<B, F>(
        &mut self,
        mut bulk_in: B,
        sample_type: SampleType,
        mut callback: F,
    ) -> Result<StreamingStats>
    where
        B: AsyncBulkInBackend,
        F: FnMut(&Transfer<'_>) -> i32,
    {
        if self.streaming {
            return Err(Error::Status(StatusCode::Busy));
        }

        self.stats = StreamingStats::default();
        self.stop_requested = false;
        self.streaming = true;

        let result = self
            .run_inner_async(&mut bulk_in, sample_type, &mut callback)
            .await;
        bulk_in.cancel_all();
        self.streaming = false;
        self.stop_requested = false;

        result.map(|()| self.stats)
    }

    fn run_inner<B, F>(
        &mut self,
        bulk_in: &mut B,
        sample_type: SampleType,
        callback: &mut F,
    ) -> Result<()>
    where
        B: BulkInBackend,
        F: FnMut(&Transfer<'_>) -> i32,
    {
        bulk_in.clear_halt()?;
        self.submit_initial_transfers(bulk_in);

        while self.streaming && !self.stop_requested {
            let Some(completion) = bulk_in.wait_next_complete(self.config.transfer_timeout) else {
                continue;
            };

            if let Some(buffer) = self.process_completion(completion, sample_type, callback)? {
                bulk_in.submit(buffer);
            }
        }

        Ok(())
    }

    async fn run_inner_async<B, F>(
        &mut self,
        bulk_in: &mut B,
        sample_type: SampleType,
        callback: &mut F,
    ) -> Result<()>
    where
        B: AsyncBulkInBackend,
        F: FnMut(&Transfer<'_>) -> i32,
    {
        bulk_in.clear_halt_async().await?;
        self.submit_initial_transfers_async(bulk_in);

        while self.streaming && !self.stop_requested {
            let completion = bulk_in.next_complete_async().await;
            if let Some(buffer) = self.process_completion(completion, sample_type, callback)? {
                bulk_in.submit(buffer);
            }
        }

        Ok(())
    }

    fn process_completion<B, F>(
        &mut self,
        completion: BulkInCompletion<B>,
        sample_type: SampleType,
        callback: &mut F,
    ) -> Result<Option<B>>
    where
        B: Deref<Target = [u8]>,
        F: FnMut(&Transfer<'_>) -> i32,
    {
        completion.status?;
        let buffer = completion.buffer;
        let actual_len = completion.actual_len;
        if actual_len != self.current_buffer_size() || actual_len > buffer.len() {
            self.stats.buffers_dropped += 1;
            return Err(Error::Status(StatusCode::LibUsb));
        }

        self.stats.buffers_received += 1;
        let sample_count = sample_count_for_buffer(actual_len, self.config.packing_enabled);
        let transfer = Transfer {
            samples: &buffer[..actual_len],
            sample_count,
            dropped_samples: self.stats.buffers_dropped * sample_count as u64,
            sample_type,
        };
        self.stats.buffers_processed += 1;

        if callback(&transfer) != 0 {
            self.stop_requested = true;
            Ok(None)
        } else {
            Ok(Some(buffer))
        }
    }

    fn submit_initial_transfers<B: BulkInBackend>(&self, bulk_in: &mut B) {
        while bulk_in.pending() < self.config.transfer_count {
            let buffer = bulk_in.allocate(self.current_buffer_size());
            bulk_in.submit(buffer);
        }
    }

    fn submit_initial_transfers_async<B: AsyncBulkInBackend>(&self, bulk_in: &mut B) {
        while bulk_in.pending() < self.config.transfer_count {
            let buffer = bulk_in.allocate(self.current_buffer_size());
            bulk_in.submit(buffer);
        }
    }
}

/// Persistent synchronous pull stream for unpacked [`SampleType::Float32Iq`] RX.
#[derive(Debug)]
pub struct DirectRxStream<B: BulkInBackend> {
    bulk_in: Option<B>,
    config: StreamingConfig,
    converter: Float32IqConverter,
    pending: Vec<(f32, f32)>,
    stats: StreamingStats,
    closed: bool,
}

/// Persistent synchronous pull stream that yields raw USB transfer blocks.
#[derive(Debug)]
pub struct RawRxStream<B: BulkInBackend> {
    bulk_in: Option<B>,
    config: StreamingConfig,
    sample_type: SampleType,
    stats: StreamingStats,
    current: Option<B::Buffer>,
    closed: bool,
}

impl<B: BulkInBackend> RawRxStream<B> {
    pub(crate) fn start(
        mut bulk_in: B,
        config: StreamingConfig,
        sample_type: SampleType,
    ) -> Result<Self> {
        bulk_in.clear_halt()?;
        while bulk_in.pending() < config.transfer_count {
            let buffer = bulk_in.allocate(config_current_buffer_size(config));
            bulk_in.submit(buffer);
        }

        Ok(Self {
            bulk_in: Some(bulk_in),
            config,
            sample_type,
            stats: StreamingStats::default(),
            current: None,
            closed: false,
        })
    }

    /// Return counters accumulated by this pull stream.
    pub fn stats(&self) -> StreamingStats {
        self.stats
    }

    /// Read the next raw transfer block.
    ///
    /// Returns `Ok(None)` when the backend times out before a block is available.
    pub fn next_transfer(&mut self) -> Result<Option<Transfer<'_>>> {
        if self.closed {
            return Err(Error::stream_closed("raw RX stream is closed"));
        }

        if let Some(buffer) = self.current.take() {
            self.bulk_in_mut()?.submit(buffer);
        }

        let timeout = self.config.transfer_timeout;
        let Some(completion) = self.bulk_in_mut()?.wait_next_complete(timeout) else {
            return Ok(None);
        };

        completion.status?;
        let buffer = completion.buffer;
        let actual_len = completion.actual_len;
        if actual_len != config_current_buffer_size(self.config) || actual_len > buffer.len() {
            self.stats.buffers_dropped += 1;
            return Err(Error::Status(StatusCode::LibUsb));
        }

        self.stats.buffers_received += 1;
        self.stats.buffers_processed += 1;
        let sample_count = sample_count_for_buffer(actual_len, self.config.packing_enabled);
        let dropped_samples = self.stats.buffers_dropped * sample_count as u64;
        self.current = Some(buffer);
        let samples = &self.current.as_ref().expect("current buffer set")[..actual_len];

        Ok(Some(Transfer {
            samples,
            sample_count,
            dropped_samples,
            sample_type: self.sample_type,
        }))
    }

    /// Close the USB queue by cancelling pending transfers.
    pub fn close(&mut self) -> StreamingStats {
        if !self.closed {
            if let Some(bulk_in) = self.bulk_in.as_mut() {
                bulk_in.cancel_all();
            }
            self.bulk_in = None;
            self.current = None;
            self.closed = true;
        }
        self.stats
    }

    fn bulk_in_mut(&mut self) -> Result<&mut B> {
        self.bulk_in
            .as_mut()
            .ok_or(Error::stream_closed("raw RX stream is closed"))
    }
}

impl<B: BulkInBackend> Drop for RawRxStream<B> {
    fn drop(&mut self) {
        let _ = self.close();
    }
}

/// Persistent async pull stream that yields raw USB transfer blocks.
#[derive(Debug)]
pub struct AsyncRawRxStream<B: AsyncBulkInBackend> {
    bulk_in: Option<B>,
    config: StreamingConfig,
    sample_type: SampleType,
    stats: StreamingStats,
    current: Option<B::Buffer>,
    closed: bool,
}

impl<B: AsyncBulkInBackend> AsyncRawRxStream<B> {
    pub(crate) async fn start(
        mut bulk_in: B,
        config: StreamingConfig,
        sample_type: SampleType,
    ) -> Result<Self> {
        bulk_in.clear_halt_async().await?;
        while bulk_in.pending() < config.transfer_count {
            let buffer = bulk_in.allocate(config_current_buffer_size(config));
            bulk_in.submit(buffer);
        }

        Ok(Self {
            bulk_in: Some(bulk_in),
            config,
            sample_type,
            stats: StreamingStats::default(),
            current: None,
            closed: false,
        })
    }

    /// Return counters accumulated by this pull stream.
    pub fn stats(&self) -> StreamingStats {
        self.stats
    }

    /// Read the next raw transfer block.
    pub async fn next_transfer(&mut self) -> Result<Option<Transfer<'_>>> {
        if self.closed {
            return Err(Error::stream_closed("async raw RX stream is closed"));
        }

        if let Some(buffer) = self.current.take() {
            self.bulk_in_mut()?.submit(buffer);
        }

        let completion = self.bulk_in_mut()?.next_complete_async().await;
        completion.status?;
        let buffer = completion.buffer;
        let actual_len = completion.actual_len;
        if actual_len != config_current_buffer_size(self.config) || actual_len > buffer.len() {
            self.stats.buffers_dropped += 1;
            return Err(Error::Status(StatusCode::LibUsb));
        }

        self.stats.buffers_received += 1;
        self.stats.buffers_processed += 1;
        let sample_count = sample_count_for_buffer(actual_len, self.config.packing_enabled);
        let dropped_samples = self.stats.buffers_dropped * sample_count as u64;
        self.current = Some(buffer);
        let samples = &self.current.as_ref().expect("current buffer set")[..actual_len];

        Ok(Some(Transfer {
            samples,
            sample_count,
            dropped_samples,
            sample_type: self.sample_type,
        }))
    }

    /// Close the USB queue by cancelling pending transfers.
    pub fn close(&mut self) -> StreamingStats {
        if !self.closed {
            if let Some(bulk_in) = self.bulk_in.as_mut() {
                bulk_in.cancel_all();
            }
            self.bulk_in = None;
            self.current = None;
            self.closed = true;
        }
        self.stats
    }

    fn bulk_in_mut(&mut self) -> Result<&mut B> {
        self.bulk_in
            .as_mut()
            .ok_or(Error::stream_closed("async raw RX stream is closed"))
    }
}

impl<B: AsyncBulkInBackend> Drop for AsyncRawRxStream<B> {
    fn drop(&mut self) {
        let _ = self.close();
    }
}

impl<B: BulkInBackend> DirectRxStream<B> {
    pub(crate) fn start(mut bulk_in: B, config: StreamingConfig) -> Result<Self> {
        bulk_in.clear_halt()?;
        while bulk_in.pending() < config.transfer_count {
            let buffer = bulk_in.allocate(config_current_buffer_size(config));
            bulk_in.submit(buffer);
        }

        Ok(Self {
            bulk_in: Some(bulk_in),
            config,
            converter: Float32IqConverter::default(),
            pending: Vec::new(),
            stats: StreamingStats::default(),
            closed: false,
        })
    }

    /// Return counters accumulated by this pull stream.
    pub fn stats(&self) -> StreamingStats {
        self.stats
    }

    /// Close the USB queue by cancelling pending transfers.
    pub fn close(&mut self) -> StreamingStats {
        if !self.closed {
            if let Some(bulk_in) = self.bulk_in.as_mut() {
                bulk_in.cancel_all();
            }
            self.bulk_in = None;
            self.pending.clear();
            self.closed = true;
        }
        self.stats
    }

    /// Read converted `(I, Q)` float samples into `out`.
    ///
    /// Returns `Ok(0)` when the backend times out before any sample is available.
    pub fn read_float32_iq(&mut self, out: &mut [(f32, f32)], timeout: Duration) -> Result<usize> {
        if self.closed {
            return Err(Error::stream_closed("direct RX stream is closed"));
        }
        if out.is_empty() {
            return Ok(0);
        }

        let mut written = 0;
        self.copy_pending(out, &mut written);
        if written == out.len() {
            return Ok(written);
        }

        loop {
            let bulk_in = self
                .bulk_in
                .as_mut()
                .ok_or(Error::stream_closed("direct RX stream is closed"))?;
            let Some(completion) = bulk_in.wait_next_complete(timeout) else {
                return Ok(written);
            };

            completion.status?;
            let buffer = completion.buffer;
            let actual_len = completion.actual_len;
            if actual_len != config_current_buffer_size(self.config) || actual_len > buffer.len() {
                self.stats.buffers_dropped += 1;
                return Err(Error::Status(StatusCode::LibUsb));
            }

            self.stats.buffers_received += 1;
            let mut converted = Vec::new();
            self.converter.process_u16le_to_f32iq(
                &buffer[..actual_len],
                self.config.decimation_factor,
                &mut converted,
            );
            self.stats.buffers_processed += 1;

            let bulk_in = self
                .bulk_in
                .as_mut()
                .ok_or(Error::stream_closed("direct RX stream is closed"))?;
            bulk_in.submit(buffer);

            self.copy_samples(&converted, out, &mut written);
            if written == out.len() {
                return Ok(written);
            }
        }
    }

    fn copy_pending(&mut self, out: &mut [(f32, f32)], written: &mut usize) {
        let take = (out.len() - *written).min(self.pending.len());
        if take == 0 {
            return;
        }

        out[*written..*written + take].copy_from_slice(&self.pending[..take]);
        self.pending.drain(..take);
        *written += take;
    }

    fn copy_samples(
        &mut self,
        samples: &[(f32, f32)],
        out: &mut [(f32, f32)],
        written: &mut usize,
    ) {
        let take = (out.len() - *written).min(samples.len());
        if take > 0 {
            out[*written..*written + take].copy_from_slice(&samples[..take]);
            *written += take;
        }
        if take < samples.len() {
            self.pending.extend_from_slice(&samples[take..]);
        }
    }
}

impl<B: BulkInBackend> Drop for DirectRxStream<B> {
    fn drop(&mut self) {
        let _ = self.close();
    }
}

fn sample_count_for_buffer(buffer_len: usize, packing_enabled: bool) -> i32 {
    if packing_enabled {
        (((buffer_len / 2) * 4) / 3) as i32
    } else {
        (buffer_len / 2) as i32
    }
}

fn config_current_buffer_size(config: StreamingConfig) -> usize {
    if config.packing_enabled {
        config.packed_buffer_size
    } else {
        config.buffer_size
    }
}
