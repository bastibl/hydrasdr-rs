//! Direct C-style streaming state machine and backend traits.

use std::future::Future;
use std::ops::Deref;
use std::time::Duration;
#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;

use crate::constants::{DEFAULT_BUFFER_SIZE, PACKED_BUFFER_SIZE};
use crate::converter::Float32IqConverter;
use crate::errors::{Error, Result};
use crate::rfone::RFONE_TRANSFER_COUNT;

/// Transfer view passed to a receive callback.
///
/// The buffer contains raw USB bytes. `sample_count` follows the C driver's byte-count formula,
/// not a final typed IQ sample abstraction.
#[derive(Debug)]
pub(crate) struct Transfer<'a> {
    pub(crate) samples: &'a [u8],
    pub(crate) sample_count: i32,
    pub(crate) dropped_samples: u64,
}

/// Counters collected during a direct streaming run.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct StreamingStats {
    /// Number of USB buffers completed by the backend.
    pub buffers_received: u64,
    /// Number of buffers successfully processed by the streaming layer.
    pub buffers_processed: u64,
    /// Number of buffers not delivered because of an error or controlled restart.
    pub buffers_dropped: u64,
}

/// Completed bulk-IN transfer from a backend.
#[derive(Debug)]
pub(crate) struct BulkInCompletion<B> {
    pub(crate) buffer: B,
    pub(crate) actual_len: usize,
    pub(crate) status: Result<()>,
}

/// Minimal synchronous bulk-IN backend used by direct streaming tests and `nusb`.
#[cfg(not(target_arch = "wasm32"))]
pub(crate) trait BulkInBackend: std::fmt::Debug {
    type Buffer: Deref<Target = [u8]>;

    fn clear_halt(&mut self) -> Result<()>;
    fn allocate(&self, len: usize) -> Self::Buffer;
    fn submit(&mut self, buffer: Self::Buffer);
    fn pending(&self) -> usize;
    fn wait_next_complete(&mut self, timeout: Duration) -> Option<BulkInCompletion<Self::Buffer>>;
    fn cancel_all(&mut self);
}

/// Provider of synchronous bulk-IN endpoints.
#[cfg(not(target_arch = "wasm32"))]
pub(crate) trait StreamingBackend: std::fmt::Debug {
    type BulkIn: BulkInBackend;

    fn bulk_in(&self, endpoint: u8) -> Result<Self::BulkIn>;
}

/// Minimal async bulk-IN backend used by direct async streaming.
pub(crate) trait AsyncBulkInBackend: std::fmt::Debug {
    type Buffer: Deref<Target = [u8]>;

    fn clear_halt_async(&mut self) -> impl Future<Output = Result<()>> + '_;
    fn allocate(&self, len: usize) -> Self::Buffer;
    fn submit(&mut self, buffer: Self::Buffer);
    fn pending(&self) -> usize;
    fn next_complete_async(&mut self) -> impl Future<Output = BulkInCompletion<Self::Buffer>> + '_;
    fn cancel_all(&mut self);
}

/// Provider of async bulk-IN endpoints.
pub(crate) trait AsyncStreamingBackend: std::fmt::Debug {
    type BulkIn: AsyncBulkInBackend;

    fn bulk_in_async(&self, endpoint: u8) -> impl Future<Output = Result<Self::BulkIn>> + '_;
}

/// C-parity streaming buffer configuration.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct StreamingConfig {
    transfer_count: usize,
    buffer_size: usize,
    packed_buffer_size: usize,
    packing_enabled: bool,
    decimation_factor: usize,
    transfer_timeout: Duration,
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

/// Mutable streaming configuration.
#[derive(Debug, Default)]
pub(crate) struct StreamingState {
    config: StreamingConfig,
}

impl StreamingState {
    /// Create an idle streaming state with C RFOne defaults.
    pub(crate) fn new() -> Self {
        Self::default()
    }

    /// Return the current streaming configuration.
    pub(crate) fn config(&self) -> StreamingConfig {
        self.config
    }

    /// Return the configured host-side DDC decimation factor.
    pub(crate) fn decimation_factor(&self) -> usize {
        self.config.decimation_factor
    }

    /// Enable or disable packed samples before streaming starts.
    pub(crate) fn set_packing(&mut self, enabled: bool) -> Result<()> {
        self.config.packing_enabled = enabled;
        Ok(())
    }

    /// Set the DDC decimation factor before streaming starts.
    pub(crate) fn set_decimation(&mut self, factor: usize) -> Result<()> {
        validate_decimation_factor(factor)?;
        self.config.decimation_factor = factor;
        Ok(())
    }
}

/// Persistent synchronous pull stream for unpacked F32 IQ RX.
#[derive(Debug)]
#[cfg(not(target_arch = "wasm32"))]
pub(crate) struct DirectRxStream<B: BulkInBackend> {
    bulk_in: Option<B>,
    config: StreamingConfig,
    converter: Float32IqConverter,
    converted: Vec<(f32, f32)>,
    converted_start: usize,
    stats: StreamingStats,
    closed: bool,
}

/// Persistent synchronous pull stream that yields raw USB transfer blocks.
#[derive(Debug)]
#[cfg(not(target_arch = "wasm32"))]
pub(crate) struct RawRxStream<B: BulkInBackend> {
    bulk_in: Option<B>,
    config: StreamingConfig,
    stats: StreamingStats,
    current: Option<B::Buffer>,
    closed: bool,
}

#[cfg(not(target_arch = "wasm32"))]
impl<B: BulkInBackend> RawRxStream<B> {
    pub(crate) fn start(mut bulk_in: B, config: StreamingConfig) -> Result<Self> {
        bulk_in.clear_halt()?;
        while bulk_in.pending() < config.transfer_count {
            let buffer = bulk_in.allocate(config_current_buffer_size(config));
            bulk_in.submit(buffer);
        }

        Ok(Self {
            bulk_in: Some(bulk_in),
            config,
            stats: StreamingStats::default(),
            current: None,
            closed: false,
        })
    }

    /// Read the next raw transfer block.
    ///
    /// Returns `Ok(None)` when the backend times out before a block is available.
    pub(crate) fn next_transfer(&mut self) -> Result<Option<Transfer<'_>>> {
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
            return Err(Error::protocol(
                "receive transfer",
                "completed with an unexpected length",
            ));
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
        }))
    }

    /// Close the USB queue, cancelling pending transfers where supported.
    pub(crate) fn close(&mut self) -> StreamingStats {
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

#[cfg(not(target_arch = "wasm32"))]
impl<B: BulkInBackend> Drop for RawRxStream<B> {
    fn drop(&mut self) {
        let _ = self.close();
    }
}

/// Persistent async pull stream that yields raw USB transfer blocks.
#[derive(Debug)]
pub(crate) struct AsyncRawRxStream<B: AsyncBulkInBackend> {
    bulk_in: Option<B>,
    config: StreamingConfig,
    stats: StreamingStats,
    current: Option<B::Buffer>,
    discard_remaining: usize,
    closed: bool,
}

/// Persistent async pull stream for unpacked F32 IQ RX.
#[derive(Debug)]
pub(crate) struct AsyncDirectRxStream<B: AsyncBulkInBackend> {
    bulk_in: Option<B>,
    config: StreamingConfig,
    converter: Float32IqConverter,
    converted: Vec<(f32, f32)>,
    converted_start: usize,
    stats: StreamingStats,
    discard_remaining: usize,
    closed: bool,
}

impl<B: AsyncBulkInBackend> AsyncRawRxStream<B> {
    pub(crate) async fn start(mut bulk_in: B, config: StreamingConfig) -> Result<Self> {
        bulk_in.clear_halt_async().await?;
        while bulk_in.pending() < config.transfer_count {
            let buffer = bulk_in.allocate(config_current_buffer_size(config));
            bulk_in.submit(buffer);
        }

        Ok(Self {
            bulk_in: Some(bulk_in),
            config,
            stats: StreamingStats::default(),
            current: None,
            discard_remaining: 0,
            closed: false,
        })
    }

    /// Read the next raw transfer block.
    pub(crate) async fn next_transfer(&mut self) -> Result<Option<Transfer<'_>>> {
        if self.closed {
            return Err(Error::stream_closed("async raw RX stream is closed"));
        }

        if let Some(buffer) = self.current.take() {
            self.bulk_in_mut()?.submit(buffer);
        }

        let (buffer, actual_len) = loop {
            let completion = self.bulk_in_mut()?.next_complete_async().await;
            if self.discard_remaining != 0 {
                self.discard_remaining -= 1;
                self.stats.buffers_received += 1;
                self.stats.buffers_dropped += 1;
                self.bulk_in_mut()?.submit(completion.buffer);
                continue;
            }
            completion.status?;
            break (completion.buffer, completion.actual_len);
        };
        if actual_len != config_current_buffer_size(self.config) || actual_len > buffer.len() {
            self.stats.buffers_dropped += 1;
            return Err(Error::protocol(
                "receive transfer",
                "completed with an unexpected length",
            ));
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
        }))
    }

    /// Preserve the endpoint queue while discarding data from before the next restart.
    pub(crate) fn pause(&mut self) -> Result<()> {
        if let Some(buffer) = self.current.take() {
            self.bulk_in_mut()?.submit(buffer);
        }
        self.discard_remaining = self
            .bulk_in
            .as_ref()
            .ok_or(Error::stream_closed("async raw RX stream is closed"))?
            .pending();
        Ok(())
    }

    pub(crate) fn stats(&self) -> StreamingStats {
        self.stats
    }

    /// Close the USB queue, cancelling pending transfers where supported.
    pub(crate) fn close(&mut self) -> StreamingStats {
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

impl<B: AsyncBulkInBackend> AsyncDirectRxStream<B> {
    pub(crate) async fn start(mut bulk_in: B, config: StreamingConfig) -> Result<Self> {
        bulk_in.clear_halt_async().await?;
        while bulk_in.pending() < config.transfer_count {
            let buffer = bulk_in.allocate(config_current_buffer_size(config));
            bulk_in.submit(buffer);
        }

        Ok(Self {
            bulk_in: Some(bulk_in),
            config,
            converter: Float32IqConverter::default(),
            converted: Vec::new(),
            converted_start: 0,
            stats: StreamingStats::default(),
            discard_remaining: 0,
            closed: false,
        })
    }

    /// Preserve the endpoint queue while discarding data from before the next restart.
    pub(crate) fn pause(&mut self) -> Result<()> {
        self.converter = Float32IqConverter::default();
        self.converted.clear();
        self.converted_start = 0;
        self.discard_remaining = self
            .bulk_in
            .as_ref()
            .ok_or(Error::stream_closed("async direct RX stream is closed"))?
            .pending();
        Ok(())
    }

    /// Update host-side decimation while preserving the existing WebUSB transfer queue.
    pub(crate) fn set_decimation_factor(&mut self, factor: usize) -> Result<()> {
        validate_decimation_factor(factor)?;
        self.config.decimation_factor = factor;
        Ok(())
    }

    #[cfg(test)]
    pub(crate) fn decimation_factor(&self) -> usize {
        self.config.decimation_factor
    }

    pub(crate) fn stats(&self) -> StreamingStats {
        self.stats
    }

    /// Close the USB queue, cancelling pending transfers where supported.
    pub(crate) fn close(&mut self) -> StreamingStats {
        if !self.closed {
            if let Some(bulk_in) = self.bulk_in.as_mut() {
                bulk_in.cancel_all();
            }
            self.bulk_in = None;
            self.converted.clear();
            self.converted_start = 0;
            self.closed = true;
        }
        self.stats
    }

    /// Read converted `(I, Q)` float samples into `out`.
    ///
    /// Each call returns after copying already-converted samples or processing one
    /// USB completion. The only await occurs before stream state is consumed, so
    /// canceling a pending read leaves the queue and buffered samples intact.
    pub(crate) async fn read_float32_iq(&mut self, out: &mut [(f32, f32)]) -> Result<usize> {
        if self.closed {
            return Err(Error::stream_closed("async direct RX stream is closed"));
        }
        if out.is_empty() {
            return Ok(0);
        }

        let mut written = 0;
        self.copy_converted(out, &mut written);
        if written != 0 {
            return Ok(written);
        }

        let (buffer, actual_len) = loop {
            let bulk_in = self
                .bulk_in
                .as_mut()
                .ok_or(Error::stream_closed("async direct RX stream is closed"))?;
            let completion = bulk_in.next_complete_async().await;
            if self.discard_remaining != 0 {
                self.discard_remaining -= 1;
                self.stats.buffers_received += 1;
                self.stats.buffers_dropped += 1;
                let bulk_in = self
                    .bulk_in
                    .as_mut()
                    .ok_or(Error::stream_closed("async direct RX stream is closed"))?;
                bulk_in.submit(completion.buffer);
                continue;
            }
            completion.status?;
            break (completion.buffer, completion.actual_len);
        };
        if actual_len != config_current_buffer_size(self.config) || actual_len > buffer.len() {
            self.stats.buffers_dropped += 1;
            return Err(Error::protocol(
                "receive transfer",
                "completed with an unexpected length",
            ));
        }

        self.stats.buffers_received += 1;
        self.converted.clear();
        self.converted_start = 0;
        self.converter.process_u16le_to_f32iq(
            &buffer[..actual_len],
            self.config.decimation_factor,
            &mut self.converted,
        );
        self.stats.buffers_processed += 1;

        let bulk_in = self
            .bulk_in
            .as_mut()
            .ok_or(Error::stream_closed("async direct RX stream is closed"))?;
        bulk_in.submit(buffer);

        self.copy_converted(out, &mut written);
        Ok(written)
    }

    fn copy_converted(&mut self, out: &mut [(f32, f32)], written: &mut usize) {
        let converted = &self.converted[self.converted_start..];
        let take = (out.len() - *written).min(converted.len());
        if take == 0 {
            return;
        }

        out[*written..*written + take].copy_from_slice(&converted[..take]);
        self.converted_start += take;
        if self.converted_start == self.converted.len() {
            self.converted.clear();
            self.converted_start = 0;
        }
        *written += take;
    }
}

impl<B: AsyncBulkInBackend> Drop for AsyncDirectRxStream<B> {
    fn drop(&mut self) {
        let _ = self.close();
    }
}

#[cfg(not(target_arch = "wasm32"))]
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
            converted: Vec::new(),
            converted_start: 0,
            stats: StreamingStats::default(),
            closed: false,
        })
    }

    /// Close the USB queue by cancelling pending transfers.
    pub(crate) fn close(&mut self) -> StreamingStats {
        if !self.closed {
            if let Some(bulk_in) = self.bulk_in.as_mut() {
                bulk_in.cancel_all();
            }
            self.bulk_in = None;
            self.converted.clear();
            self.converted_start = 0;
            self.closed = true;
        }
        self.stats
    }

    /// Read converted `(I, Q)` float samples into `out`.
    ///
    /// Returns `Ok(0)` when the backend times out before any sample is available.
    /// `timeout` is a total deadline for this read call, not a per-transfer timeout.
    pub(crate) fn read_float32_iq(
        &mut self,
        out: &mut [(f32, f32)],
        timeout: Duration,
    ) -> Result<usize> {
        if self.closed {
            return Err(Error::stream_closed("direct RX stream is closed"));
        }
        if out.is_empty() {
            return Ok(0);
        }

        let mut written = 0;
        self.copy_converted(out, &mut written);
        if written == out.len() {
            return Ok(written);
        }

        let deadline = Instant::now().checked_add(timeout);
        loop {
            let bulk_in = self
                .bulk_in
                .as_mut()
                .ok_or(Error::stream_closed("direct RX stream is closed"))?;
            let Some(completion) = bulk_in.wait_next_complete(remaining_timeout(deadline, timeout))
            else {
                return Ok(written);
            };

            completion.status?;
            let buffer = completion.buffer;
            let actual_len = completion.actual_len;
            if actual_len != config_current_buffer_size(self.config) || actual_len > buffer.len() {
                self.stats.buffers_dropped += 1;
                return Err(Error::protocol(
                    "receive transfer",
                    "completed with an unexpected length",
                ));
            }

            self.stats.buffers_received += 1;
            self.converted.clear();
            self.converted_start = 0;
            self.converter.process_u16le_to_f32iq(
                &buffer[..actual_len],
                self.config.decimation_factor,
                &mut self.converted,
            );
            self.stats.buffers_processed += 1;

            let bulk_in = self
                .bulk_in
                .as_mut()
                .ok_or(Error::stream_closed("direct RX stream is closed"))?;
            bulk_in.submit(buffer);

            self.copy_converted(out, &mut written);
            if written == out.len() {
                return Ok(written);
            }
        }
    }

    fn copy_converted(&mut self, out: &mut [(f32, f32)], written: &mut usize) {
        let converted = &self.converted[self.converted_start..];
        let take = (out.len() - *written).min(converted.len());
        if take == 0 {
            return;
        }

        out[*written..*written + take].copy_from_slice(&converted[..take]);
        self.converted_start += take;
        if self.converted_start == self.converted.len() {
            self.converted.clear();
            self.converted_start = 0;
        }
        *written += take;
    }
}

#[cfg(not(target_arch = "wasm32"))]
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

fn validate_decimation_factor(factor: usize) -> Result<()> {
    if !matches!(factor, 1 | 2 | 4 | 8 | 16 | 32 | 64) {
        return Err(Error::invalid_config(
            "decimation_factor",
            "must be one of 1, 2, 4, 8, 16, 32, or 64",
        ));
    }
    Ok(())
}

#[cfg(not(target_arch = "wasm32"))]
fn remaining_timeout(deadline: Option<Instant>, fallback: Duration) -> Duration {
    deadline.map_or(fallback, |deadline| {
        deadline.saturating_duration_since(Instant::now())
    })
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;

    use futures_lite::future::block_on;

    use super::*;

    #[derive(Debug, Default)]
    struct FakeAsyncBulkIn {
        submitted: VecDeque<Vec<u8>>,
        submit_count: usize,
        cancelled: bool,
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
            self.submit_count += 1;
            self.submitted.push_back(buffer);
        }

        fn pending(&self) -> usize {
            self.submitted.len()
        }

        async fn next_complete_async(&mut self) -> BulkInCompletion<Self::Buffer> {
            let buffer = self
                .submitted
                .pop_front()
                .expect("fake async bulk queue should contain a submitted buffer");
            BulkInCompletion {
                actual_len: buffer.len(),
                buffer,
                status: Ok(()),
            }
        }

        fn cancel_all(&mut self) {
            self.cancelled = true;
            self.submitted.clear();
        }
    }

    #[test]
    fn async_f32_read_resubmits_completed_buffer_before_returning() {
        block_on(async {
            let bulk_in = FakeAsyncBulkIn::default();
            let mut stream = AsyncDirectRxStream::start(bulk_in, StreamingConfig::default())
                .await
                .expect("start fake async stream");
            let initial_submit_count = stream.bulk_in.as_ref().expect("bulk in").submit_count;
            assert_eq!(initial_submit_count, RFONE_TRANSFER_COUNT as usize);

            let mut out = [(0.0, 0.0); 1];
            let read = stream
                .read_float32_iq(&mut out)
                .await
                .expect("read converted samples");

            let bulk_in = stream.bulk_in.as_ref().expect("bulk in");
            assert_eq!(read, out.len());
            assert_eq!(bulk_in.submit_count, initial_submit_count + 1);
            assert_eq!(bulk_in.pending(), RFONE_TRANSFER_COUNT as usize);
        });
    }

    #[test]
    fn async_f32_read_returns_buffered_samples_without_waiting_for_another_completion() {
        block_on(async {
            let bulk_in = FakeAsyncBulkIn::default();
            let mut stream = AsyncDirectRxStream::start(bulk_in, StreamingConfig::default())
                .await
                .expect("start fake async stream");

            let mut first = [(0.0, 0.0); 1];
            assert_eq!(
                stream
                    .read_float32_iq(&mut first)
                    .await
                    .expect("first read"),
                1
            );
            let buffered = stream.converted.len() - stream.converted_start;
            assert!(buffered > 0);
            assert_eq!(stream.stats.buffers_received, 1);

            let mut out = vec![(0.0, 0.0); buffered + 1];
            assert_eq!(
                stream
                    .read_float32_iq(&mut out)
                    .await
                    .expect("buffered read"),
                buffered
            );
            assert_eq!(stream.stats.buffers_received, 1);
        });
    }

    #[test]
    fn persistent_async_f32_stream_accepts_processing_config_updates() {
        block_on(async {
            let bulk_in = FakeAsyncBulkIn::default();
            let mut stream = AsyncDirectRxStream::start(bulk_in, StreamingConfig::default())
                .await
                .expect("start fake async stream");
            stream.pause().expect("pause fake async stream");

            stream
                .set_decimation_factor(4)
                .expect("update persistent stream decimation");

            assert_eq!(stream.config.decimation_factor, 4);
            assert_eq!(
                stream.bulk_in.as_ref().expect("bulk in").pending(),
                RFONE_TRANSFER_COUNT as usize
            );
        });
    }
}
