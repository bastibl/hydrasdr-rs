use std::future::Future;
use std::ops::Deref;
use std::time::Duration;

use crate::constants::{DEFAULT_BUFFER_SIZE, PACKED_BUFFER_SIZE};
use crate::errors::{Error, Result, StatusCode};
use crate::rfone::RFONE_TRANSFER_COUNT;
use crate::types::SampleType;

#[derive(Debug)]
pub struct Transfer<'a> {
    pub samples: &'a [u8],
    pub sample_count: i32,
    pub dropped_samples: u64,
    pub sample_type: SampleType,
}

pub type SampleBlockCallback = dyn FnMut(&Transfer<'_>) -> i32 + Send;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct StreamingStats {
    pub buffers_received: u64,
    pub buffers_processed: u64,
    pub buffers_dropped: u64,
}

#[derive(Debug)]
pub struct BulkInCompletion<B> {
    pub buffer: B,
    pub actual_len: usize,
    pub status: Result<()>,
}

pub trait BulkInBackend: std::fmt::Debug {
    type Buffer: Deref<Target = [u8]>;

    fn clear_halt(&mut self) -> Result<()>;
    fn allocate(&self, len: usize) -> Self::Buffer;
    fn submit(&mut self, buffer: Self::Buffer);
    fn pending(&self) -> usize;
    fn wait_next_complete(&mut self, timeout: Duration) -> Option<BulkInCompletion<Self::Buffer>>;
    fn cancel_all(&mut self);
}

pub trait StreamingBackend: std::fmt::Debug {
    type BulkIn: BulkInBackend;

    fn bulk_in(&self, endpoint: u8) -> Result<Self::BulkIn>;
}

pub trait AsyncBulkInBackend: std::fmt::Debug {
    type Buffer: Deref<Target = [u8]>;

    fn clear_halt_async(&mut self) -> impl Future<Output = Result<()>> + '_;
    fn allocate(&self, len: usize) -> Self::Buffer;
    fn submit(&mut self, buffer: Self::Buffer);
    fn pending(&self) -> usize;
    fn next_complete_async(&mut self) -> impl Future<Output = BulkInCompletion<Self::Buffer>> + '_;
    fn cancel_all(&mut self);
}

pub trait AsyncStreamingBackend: std::fmt::Debug {
    type BulkIn: AsyncBulkInBackend;

    fn bulk_in_async(&self, endpoint: u8) -> impl Future<Output = Result<Self::BulkIn>> + '_;
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct StreamingConfig {
    pub transfer_count: usize,
    pub buffer_size: usize,
    pub packed_buffer_size: usize,
    pub packing_enabled: bool,
    pub transfer_timeout: Duration,
}

impl Default for StreamingConfig {
    fn default() -> Self {
        Self {
            transfer_count: RFONE_TRANSFER_COUNT as usize,
            buffer_size: DEFAULT_BUFFER_SIZE,
            packed_buffer_size: PACKED_BUFFER_SIZE,
            packing_enabled: false,
            transfer_timeout: Duration::MAX,
        }
    }
}

#[derive(Debug, Default)]
pub struct StreamingState {
    config: StreamingConfig,
    streaming: bool,
    stop_requested: bool,
    stats: StreamingStats,
}

impl StreamingState {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn stats(&self) -> StreamingStats {
        self.stats
    }

    pub fn is_streaming(&self) -> bool {
        self.streaming
    }

    pub fn request_stop(&mut self) {
        self.stop_requested = true;
    }

    pub fn set_packing(&mut self, enabled: bool) -> Result<()> {
        if self.streaming {
            return Err(Error::Status(StatusCode::Busy));
        }
        self.config.packing_enabled = enabled;
        Ok(())
    }

    pub fn current_buffer_size(&self) -> usize {
        if self.config.packing_enabled {
            self.config.packed_buffer_size
        } else {
            self.config.buffer_size
        }
    }

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

fn sample_count_for_buffer(buffer_len: usize, packing_enabled: bool) -> i32 {
    if packing_enabled {
        (((buffer_len / 2) * 4) / 3) as i32
    } else {
        (buffer_len / 2) as i32
    }
}
