//! Direct C-style streaming state machine and backend traits.

use std::future::Future;
use std::ops::Deref;
#[cfg(not(target_arch = "wasm32"))]
use std::time::{Duration, Instant};

use crate::Complex32;
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
///
/// These counters describe USB completions observed by the host. RFOne bulk data
/// has no sequence number, so the driver cannot detect samples lost in the device
/// before a USB transfer completes (for example, when the application stops
/// polling long enough to exhaust the host transfer queue).
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct StreamingStats {
    /// Number of USB completions consumed by the driver.
    pub buffers_received: u64,
    /// Number of buffers successfully processed by the streaming layer.
    pub buffers_processed: u64,
    /// Number of observed completions not delivered because of an error or controlled restart.
    ///
    /// This does not include device-side loss that happened before USB completion.
    pub buffers_dropped: u64,
    /// Number of dropped buffers that belonged to the queue retained across a restart.
    ///
    /// This is a subset of [`StreamingStats::buffers_dropped`]. On backends without
    /// transfer cancellation, including WebUSB, consuming these old submissions may
    /// delay the first fresh block after restarting.
    pub buffers_discarded_on_restart: u64,
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

    fn bulk_in(&self, endpoint: u8) -> Result<Self::BulkIn>;
}

/// C-parity streaming buffer configuration.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct StreamingConfig {
    transfer_count: usize,
    buffer_size: usize,
    packed_buffer_size: usize,
    packing_enabled: bool,
    decimation_factor: usize,
}

impl Default for StreamingConfig {
    fn default() -> Self {
        Self {
            transfer_count: RFONE_TRANSFER_COUNT as usize,
            buffer_size: DEFAULT_BUFFER_SIZE,
            packed_buffer_size: PACKED_BUFFER_SIZE,
            packing_enabled: false,
            decimation_factor: 1,
        }
    }
}

#[derive(Debug)]
pub(crate) struct PreparedBulkIn<B, T> {
    bulk_in: B,
    buffers: Vec<T>,
    config: StreamingConfig,
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
    pub(crate) fn set_packing(&mut self, enabled: bool) {
        self.config.packing_enabled = enabled;
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
    current: Option<B::Buffer>,
    current_len: usize,
    current_offset: usize,
    pending_iq: Option<Complex32>,
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
    pub(crate) fn prepare(bulk_in: B, config: StreamingConfig) -> PreparedBulkIn<B, B::Buffer> {
        prepare_bulk_in(bulk_in, config)
    }

    /// Read the next raw transfer block.
    ///
    /// Returns `Ok(None)` when the backend times out before a block is available.
    pub(crate) fn next_transfer(&mut self, timeout: Duration) -> Result<Option<Transfer<'_>>> {
        if self.closed {
            return Err(Error::stream_closed("raw RX stream is closed"));
        }

        if let Some(buffer) = self.current.take() {
            self.bulk_in_mut()?.submit(buffer);
        }

        let Some(completion) = self.bulk_in_mut()?.wait_next_complete(timeout) else {
            return Ok(None);
        };

        self.stats.buffers_received += 1;
        let (buffer, actual_len) =
            match checked_completion(completion, config_current_buffer_size(self.config)) {
                Ok(completion) => completion,
                Err((_buffer, error)) => {
                    self.stats.buffers_dropped += 1;
                    self.close();
                    return Err(error);
                }
            };

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
    current: Option<B::Buffer>,
    current_len: usize,
    current_offset: usize,
    pending_iq: Option<Complex32>,
    stats: StreamingStats,
    discard_remaining: usize,
    closed: bool,
}

impl<B: AsyncBulkInBackend> AsyncRawRxStream<B> {
    pub(crate) fn prepare(bulk_in: B, config: StreamingConfig) -> PreparedBulkIn<B, B::Buffer> {
        prepare_async_bulk_in(bulk_in, config)
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
                self.stats.buffers_discarded_on_restart += 1;
                self.bulk_in_mut()?.submit(completion.buffer);
                continue;
            }
            self.stats.buffers_received += 1;
            match checked_completion(completion, config_current_buffer_size(self.config)) {
                Ok(completion) => break completion,
                Err((_buffer, error)) => {
                    self.stats.buffers_dropped += 1;
                    self.close();
                    return Err(error);
                }
            }
        };

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
        if self.closed {
            return Ok(());
        }
        if let Some(buffer) = self.current.take() {
            self.bulk_in_mut()?.submit(buffer);
        }
        let bulk_in = self
            .bulk_in
            .as_mut()
            .ok_or(Error::stream_closed("async raw RX stream is closed"))?;
        self.discard_remaining = bulk_in.pending();
        bulk_in.cancel_all();
        Ok(())
    }

    pub(crate) fn stats(&self) -> StreamingStats {
        self.stats
    }

    pub(crate) fn is_closed(&self) -> bool {
        self.closed
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
    pub(crate) fn prepare(bulk_in: B, config: StreamingConfig) -> PreparedBulkIn<B, B::Buffer> {
        prepare_async_bulk_in(bulk_in, config)
    }

    /// Preserve the endpoint queue while discarding data from before the next restart.
    pub(crate) fn pause(&mut self) -> Result<()> {
        self.converter = Float32IqConverter::default();
        self.pending_iq = None;
        self.current_len = 0;
        self.current_offset = 0;
        if let Some(buffer) = self.current.take() {
            self.bulk_in
                .as_mut()
                .ok_or(Error::stream_closed("async direct RX stream is closed"))?
                .submit(buffer);
        }
        let bulk_in = self
            .bulk_in
            .as_mut()
            .ok_or(Error::stream_closed("async direct RX stream is closed"))?;
        self.discard_remaining = bulk_in.pending();
        bulk_in.cancel_all();
        Ok(())
    }

    /// Update host-side decimation while preserving the existing WebUSB transfer queue.
    pub(crate) fn set_decimation_factor(&mut self, factor: usize) -> Result<()> {
        validate_decimation_factor(factor)?;
        self.config.decimation_factor = factor;
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
            self.current_len = 0;
            self.current_offset = 0;
            self.pending_iq = None;
            self.closed = true;
        }
        self.stats
    }

    /// Read converted complex float samples into `out`.
    ///
    /// Each call drains samples already buffered in the stream, or awaits and
    /// processes at most one new USB completion. The only await occurs before
    /// stream state is consumed, so canceling a pending read leaves the queue and
    /// buffered samples intact.
    pub(crate) async fn read_float32_iq(&mut self, out: &mut [Complex32]) -> Result<usize> {
        if self.closed {
            return Err(Error::stream_closed("async direct RX stream is closed"));
        }
        if out.is_empty() {
            return Ok(0);
        }

        let mut written = 0;
        if let Some(sample) = self.pending_iq.take() {
            out[written] = sample;
            written += 1;
            if written == out.len() {
                return Ok(written);
            }
        }

        loop {
            if self.current.is_none() {
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
                        self.stats.buffers_discarded_on_restart += 1;
                        self.bulk_in
                            .as_mut()
                            .ok_or(Error::stream_closed("async direct RX stream is closed"))?
                            .submit(completion.buffer);
                        continue;
                    }
                    self.stats.buffers_received += 1;
                    match checked_completion(completion, config_current_buffer_size(self.config)) {
                        Ok(completion) => break completion,
                        Err((_buffer, error)) => {
                            self.stats.buffers_dropped += 1;
                            self.close();
                            return Err(error);
                        }
                    }
                };
                self.current = Some(buffer);
                self.current_len = actual_len;
                self.current_offset = 0;
                self.stats.buffers_processed += 1;
            }

            let buffer = self.current.as_ref().expect("current buffer set");
            let (consumed, produced, pending) = self.converter.process_u16le_to_f32iq_slice(
                &buffer[self.current_offset..self.current_len],
                self.config.decimation_factor,
                &mut out[written..],
            );
            self.current_offset += consumed;
            written += produced;
            self.pending_iq = pending;

            if self.current_offset == self.current_len {
                let buffer = self.current.take().expect("current buffer set");
                self.bulk_in
                    .as_mut()
                    .ok_or(Error::stream_closed("async direct RX stream is closed"))?
                    .submit(buffer);
                self.current_len = 0;
                self.current_offset = 0;
                return Ok(written);
            } else if consumed == 0 {
                return Err(Error::protocol(
                    "convert F32 IQ samples",
                    "USB buffer ended with an incomplete conversion group",
                ));
            }

            if written == out.len() {
                return Ok(written);
            }
        }
    }
}

impl<B: AsyncBulkInBackend> Drop for AsyncDirectRxStream<B> {
    fn drop(&mut self) {
        let _ = self.close();
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl<B: BulkInBackend> DirectRxStream<B> {
    pub(crate) fn prepare(bulk_in: B, config: StreamingConfig) -> PreparedBulkIn<B, B::Buffer> {
        prepare_bulk_in(bulk_in, config)
    }

    /// Close the USB queue by cancelling pending transfers.
    pub(crate) fn close(&mut self) -> StreamingStats {
        if !self.closed {
            if let Some(bulk_in) = self.bulk_in.as_mut() {
                bulk_in.cancel_all();
            }
            self.bulk_in = None;
            self.current = None;
            self.current_len = 0;
            self.current_offset = 0;
            self.pending_iq = None;
            self.closed = true;
        }
        self.stats
    }

    /// Read converted complex float samples into `out`.
    ///
    /// Returns `Ok(0)` when the backend times out before any sample is available.
    /// `timeout` is a total deadline for this read call, not a per-transfer timeout.
    pub(crate) fn read_float32_iq(
        &mut self,
        out: &mut [Complex32],
        timeout: Duration,
    ) -> Result<usize> {
        if self.closed {
            return Err(Error::stream_closed("direct RX stream is closed"));
        }
        if out.is_empty() {
            return Ok(0);
        }

        let mut written = 0;
        if let Some(sample) = self.pending_iq.take() {
            out[written] = sample;
            written += 1;
            if written == out.len() {
                return Ok(written);
            }
        }
        let deadline = Instant::now().checked_add(timeout);
        loop {
            if self.current.is_none() {
                let bulk_in = self
                    .bulk_in
                    .as_mut()
                    .ok_or(Error::stream_closed("direct RX stream is closed"))?;
                let Some(completion) =
                    bulk_in.wait_next_complete(remaining_timeout(deadline, timeout))
                else {
                    return Ok(written);
                };

                self.stats.buffers_received += 1;
                let (buffer, actual_len) =
                    match checked_completion(completion, config_current_buffer_size(self.config)) {
                        Ok(completion) => completion,
                        Err((_buffer, error)) => {
                            self.stats.buffers_dropped += 1;
                            self.close();
                            return Err(error);
                        }
                    };
                self.current = Some(buffer);
                self.current_len = actual_len;
                self.current_offset = 0;
                self.stats.buffers_processed += 1;
            }

            let buffer = self.current.as_ref().expect("current buffer set");
            let (consumed, produced, pending) = self.converter.process_u16le_to_f32iq_slice(
                &buffer[self.current_offset..self.current_len],
                self.config.decimation_factor,
                &mut out[written..],
            );
            self.current_offset += consumed;
            written += produced;
            self.pending_iq = pending;

            if self.current_offset == self.current_len {
                let buffer = self.current.take().expect("current buffer set");
                self.bulk_in
                    .as_mut()
                    .ok_or(Error::stream_closed("direct RX stream is closed"))?
                    .submit(buffer);
                self.current_len = 0;
                self.current_offset = 0;
            } else if consumed == 0 {
                return Err(Error::protocol(
                    "convert F32 IQ samples",
                    "USB buffer ended with an incomplete conversion group",
                ));
            }

            if written == out.len() {
                return Ok(written);
            }
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl<B: BulkInBackend> Drop for DirectRxStream<B> {
    fn drop(&mut self) {
        let _ = self.close();
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl<B: BulkInBackend> PreparedBulkIn<B, B::Buffer> {
    pub(crate) fn start_raw(mut self) -> Result<RawRxStream<B>> {
        self.bulk_in.clear_halt()?;
        for buffer in self.buffers {
            self.bulk_in.submit(buffer);
        }

        Ok(RawRxStream {
            bulk_in: Some(self.bulk_in),
            config: self.config,
            stats: StreamingStats::default(),
            current: None,
            closed: false,
        })
    }

    pub(crate) fn start_direct(mut self) -> Result<DirectRxStream<B>> {
        self.bulk_in.clear_halt()?;
        for buffer in self.buffers {
            self.bulk_in.submit(buffer);
        }

        Ok(DirectRxStream {
            bulk_in: Some(self.bulk_in),
            config: self.config,
            converter: Float32IqConverter::default(),
            current: None,
            current_len: 0,
            current_offset: 0,
            pending_iq: None,
            stats: StreamingStats::default(),
            closed: false,
        })
    }
}

impl<B: AsyncBulkInBackend> PreparedBulkIn<B, B::Buffer> {
    pub(crate) async fn start_async_raw(mut self) -> Result<AsyncRawRxStream<B>> {
        self.bulk_in.clear_halt_async().await?;
        for buffer in self.buffers {
            self.bulk_in.submit(buffer);
        }

        Ok(AsyncRawRxStream {
            bulk_in: Some(self.bulk_in),
            config: self.config,
            stats: StreamingStats::default(),
            current: None,
            discard_remaining: 0,
            closed: false,
        })
    }

    pub(crate) async fn start_async_direct(mut self) -> Result<AsyncDirectRxStream<B>> {
        self.bulk_in.clear_halt_async().await?;
        for buffer in self.buffers {
            self.bulk_in.submit(buffer);
        }

        Ok(AsyncDirectRxStream {
            bulk_in: Some(self.bulk_in),
            config: self.config,
            converter: Float32IqConverter::default(),
            current: None,
            current_len: 0,
            current_offset: 0,
            pending_iq: None,
            stats: StreamingStats::default(),
            discard_remaining: 0,
            closed: false,
        })
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn prepare_bulk_in<B: BulkInBackend>(
    bulk_in: B,
    config: StreamingConfig,
) -> PreparedBulkIn<B, B::Buffer> {
    let missing = config.transfer_count.saturating_sub(bulk_in.pending());
    let buffer_size = config_current_buffer_size(config);
    let buffers = (0..missing)
        .map(|_| bulk_in.allocate(buffer_size))
        .collect();
    PreparedBulkIn {
        bulk_in,
        buffers,
        config,
    }
}

fn prepare_async_bulk_in<B: AsyncBulkInBackend>(
    bulk_in: B,
    config: StreamingConfig,
) -> PreparedBulkIn<B, B::Buffer> {
    let missing = config.transfer_count.saturating_sub(bulk_in.pending());
    let buffer_size = config_current_buffer_size(config);
    let buffers = (0..missing)
        .map(|_| bulk_in.allocate(buffer_size))
        .collect();
    PreparedBulkIn {
        bulk_in,
        buffers,
        config,
    }
}

fn sample_count_for_buffer(buffer_len: usize, packing_enabled: bool) -> i32 {
    if packing_enabled {
        (((buffer_len / 2) * 4) / 3) as i32
    } else {
        (buffer_len / 2) as i32
    }
}

fn checked_completion<B: Deref<Target = [u8]>>(
    completion: BulkInCompletion<B>,
    expected_len: usize,
) -> core::result::Result<(B, usize), (B, Error)> {
    let BulkInCompletion {
        buffer,
        actual_len,
        status,
    } = completion;
    if let Err(error) = status {
        return Err((buffer, error));
    }
    if actual_len != expected_len || actual_len > buffer.len() {
        return Err((
            buffer,
            Error::protocol("receive transfer", "completed with an unexpected length"),
        ));
    }
    Ok((buffer, actual_len))
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

    #[cfg(not(target_arch = "wasm32"))]
    #[derive(Debug, Default)]
    struct FakeBulkIn {
        submitted: VecDeque<Vec<u8>>,
        last_timeout: Option<Duration>,
        cancelled: bool,
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
            timeout: Duration,
        ) -> Option<BulkInCompletion<Self::Buffer>> {
            self.last_timeout = Some(timeout);
            None
        }

        fn cancel_all(&mut self) {
            self.cancelled = true;
        }
    }

    #[derive(Debug, Default)]
    struct FakeAsyncBulkIn {
        submitted: VecDeque<Vec<u8>>,
        submit_count: usize,
        fail_next: bool,
        short_next: bool,
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
            let status = if self.fail_next {
                self.fail_next = false;
                Err(Error::from(nusb::transfer::TransferError::Fault))
            } else {
                Ok(())
            };
            let actual_len = if self.short_next {
                self.short_next = false;
                buffer.len() - 2
            } else {
                buffer.len()
            };
            BulkInCompletion {
                actual_len,
                buffer,
                status,
            }
        }

        fn cancel_all(&mut self) {
            self.cancelled = true;
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn raw_read_forwards_timeout_without_closing_queue() {
        let timeout = Duration::from_millis(25);
        let bulk_in = FakeBulkIn::default();
        let mut stream = RawRxStream::prepare(bulk_in, StreamingConfig::default())
            .start_raw()
            .expect("start fake raw stream");

        assert!(
            stream
                .next_transfer(timeout)
                .expect("timed raw read")
                .is_none()
        );

        let bulk_in = stream.bulk_in.as_ref().expect("bulk in");
        assert_eq!(bulk_in.last_timeout, Some(timeout));
        assert_eq!(bulk_in.pending(), RFONE_TRANSFER_COUNT as usize);
        assert!(!bulk_in.cancelled);
    }

    #[test]
    fn async_f32_read_retains_partially_consumed_usb_buffer() {
        block_on(async {
            let bulk_in = FakeAsyncBulkIn::default();
            let prepared = AsyncDirectRxStream::prepare(bulk_in, StreamingConfig::default());
            assert_eq!(prepared.buffers.len(), RFONE_TRANSFER_COUNT as usize);
            assert_eq!(prepared.bulk_in.pending(), 0);

            let mut stream = prepared
                .start_async_direct()
                .await
                .expect("start fake async stream");
            let initial_submit_count = stream.bulk_in.as_ref().expect("bulk in").submit_count;
            assert_eq!(initial_submit_count, RFONE_TRANSFER_COUNT as usize);

            let mut out = [Complex32::default(); 1];
            let read = stream
                .read_float32_iq(&mut out)
                .await
                .expect("read converted samples");

            let bulk_in = stream.bulk_in.as_ref().expect("bulk in");
            assert_eq!(read, out.len());
            assert_eq!(bulk_in.submit_count, initial_submit_count);
            assert_eq!(bulk_in.pending(), RFONE_TRANSFER_COUNT as usize - 1);
            assert!(stream.current.is_some());
        });
    }

    #[test]
    fn async_f32_read_resubmits_a_fully_consumed_usb_buffer() {
        block_on(async {
            let bulk_in = FakeAsyncBulkIn::default();
            let mut stream = AsyncDirectRxStream::prepare(bulk_in, StreamingConfig::default())
                .start_async_direct()
                .await
                .expect("start fake async stream");
            let initial_submit_count = stream.bulk_in.as_ref().expect("bulk in").submit_count;
            let mut out = vec![Complex32::default(); DEFAULT_BUFFER_SIZE / 4];

            assert_eq!(
                stream
                    .read_float32_iq(&mut out)
                    .await
                    .expect("read one full converted transfer"),
                out.len()
            );

            let bulk_in = stream.bulk_in.as_ref().expect("bulk in");
            assert_eq!(bulk_in.submit_count, initial_submit_count + 1);
            assert_eq!(bulk_in.pending(), RFONE_TRANSFER_COUNT as usize);
            assert!(stream.current.is_none());
            assert!(stream.pending_iq.is_none());
        });
    }

    #[test]
    fn async_f32_read_waits_for_at_most_one_new_usb_completion() {
        block_on(async {
            let bulk_in = FakeAsyncBulkIn::default();
            let mut stream = AsyncDirectRxStream::prepare(bulk_in, StreamingConfig::default())
                .start_async_direct()
                .await
                .expect("start fake async stream");
            let samples_per_transfer = DEFAULT_BUFFER_SIZE / 4;
            let mut out = vec![Complex32::default(); samples_per_transfer + 1];

            assert_eq!(
                stream
                    .read_float32_iq(&mut out)
                    .await
                    .expect("read one converted transfer"),
                samples_per_transfer
            );
            assert_eq!(stream.stats.buffers_received, 1);
        });
    }

    #[test]
    fn async_raw_blocks_reuse_the_fixed_transfer_pool() {
        block_on(async {
            let bulk_in = FakeAsyncBulkIn::default();
            let mut stream = AsyncRawRxStream::prepare(bulk_in, StreamingConfig::default())
                .start_async_raw()
                .await
                .expect("start fake async raw stream");
            let initial_submit_count = stream.bulk_in.as_ref().expect("bulk in").submit_count;

            {
                let transfer = stream
                    .next_transfer()
                    .await
                    .expect("read first raw transfer")
                    .expect("raw transfer");
                assert_eq!(transfer.samples.len(), DEFAULT_BUFFER_SIZE);
            }
            assert_eq!(
                stream.bulk_in.as_ref().expect("bulk in").submit_count,
                initial_submit_count
            );

            let _second = stream
                .next_transfer()
                .await
                .expect("read second raw transfer")
                .expect("raw transfer");
            let bulk_in = stream.bulk_in.as_ref().expect("bulk in");
            assert_eq!(bulk_in.submit_count, initial_submit_count + 1);
            assert_eq!(bulk_in.pending(), RFONE_TRANSFER_COUNT as usize - 1);
        });
    }

    #[test]
    fn async_f32_read_returns_buffered_samples_without_waiting_for_another_completion() {
        block_on(async {
            let bulk_in = FakeAsyncBulkIn::default();
            let mut stream = AsyncDirectRxStream::prepare(bulk_in, StreamingConfig::default())
                .start_async_direct()
                .await
                .expect("start fake async stream");

            let mut first = [Complex32::default(); 1];
            assert_eq!(
                stream
                    .read_float32_iq(&mut first)
                    .await
                    .expect("first read"),
                1
            );
            assert!(stream.current.is_some());
            assert!(stream.pending_iq.is_some());
            assert_eq!(stream.stats.buffers_received, 1);

            let mut out = [Complex32::default(); 1];
            assert_eq!(
                stream
                    .read_float32_iq(&mut out)
                    .await
                    .expect("buffered read"),
                1
            );
            assert_eq!(stream.stats.buffers_received, 1);
        });
    }

    #[test]
    fn async_f32_read_closes_stream_after_failed_completion() {
        block_on(async {
            let bulk_in = FakeAsyncBulkIn::default();
            let mut stream = AsyncDirectRxStream::prepare(bulk_in, StreamingConfig::default())
                .start_async_direct()
                .await
                .expect("start fake async stream");
            stream.bulk_in.as_mut().expect("bulk in").fail_next = true;

            let error = stream
                .read_float32_iq(&mut [Complex32::default(); 1])
                .await
                .expect_err("failed completion must be reported");

            assert!(matches!(error, Error::Transfer(_)));
            assert_eq!(stream.stats.buffers_received, 1);
            assert_eq!(stream.stats.buffers_dropped, 1);
            assert!(stream.closed);
            assert!(stream.bulk_in.is_none());
        });
    }

    #[test]
    fn async_f32_read_closes_stream_after_short_completion() {
        block_on(async {
            let bulk_in = FakeAsyncBulkIn::default();
            let mut stream = AsyncDirectRxStream::prepare(bulk_in, StreamingConfig::default())
                .start_async_direct()
                .await
                .expect("start fake async stream");
            stream.bulk_in.as_mut().expect("bulk in").short_next = true;

            let error = stream
                .read_float32_iq(&mut [Complex32::default(); 1])
                .await
                .expect_err("short completion must be reported");

            assert!(matches!(error, Error::Protocol { .. }));
            assert_eq!(stream.stats.buffers_received, 1);
            assert_eq!(stream.stats.buffers_dropped, 1);
            assert!(stream.closed);
            assert!(stream.bulk_in.is_none());
        });
    }

    #[test]
    fn persistent_async_f32_stream_accepts_processing_config_updates() {
        block_on(async {
            let bulk_in = FakeAsyncBulkIn::default();
            let mut stream = AsyncDirectRxStream::prepare(bulk_in, StreamingConfig::default())
                .start_async_direct()
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
