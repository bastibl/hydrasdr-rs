//! Sample-mode-specific state, independent of USB completion scheduling.

use super::queue::Queue;
use crate::{
    Complex32,
    converter::Float32IqConverter,
    errors::{Error, Result},
    maybe_future::NonWasmSend,
    usb::BulkInBackend,
};

pub trait Processor: Default + NonWasmSend + 'static {
    fn reset(&mut self);
}

#[derive(Default)]
pub struct RawProcessor;
impl Processor for RawProcessor {
    fn reset(&mut self) {}
}

#[derive(Default)]
pub struct IqProcessor {
    converter: Float32IqConverter,
    offset: usize,
    pending: Option<Complex32>,
}
impl Processor for IqProcessor {
    fn reset(&mut self) {
        *self = Self::default();
    }
}
impl IqProcessor {
    pub(super) fn has_pending(&self) -> bool {
        self.pending.is_some()
    }
    pub(super) fn process<B: BulkInBackend>(
        &mut self,
        queue: &mut Queue<B>,
        out: &mut [Complex32],
    ) -> Result<usize> {
        if out.is_empty() {
            return Ok(0);
        }
        let mut written = 0;
        if let Some(sample) = self.pending.take() {
            out[0] = sample;
            written = 1;
        }
        while written < out.len() {
            let Some(buffer) = queue.current.as_ref() else {
                break;
            };
            let progress = self.converter.process_u16le_to_f32iq_slice(
                &buffer[self.offset..queue.current_len],
                queue.settings.decimation,
                &mut out[written..],
            );
            self.offset += progress.consumed_bytes;
            written += progress.written;
            self.pending = progress.pending;
            if self.offset == queue.current_len {
                queue.release_current();
                self.offset = 0;
                break;
            }
            if progress.consumed_bytes == 0 {
                return Err(Error::protocol(
                    "convert F32 IQ samples",
                    "USB buffer ended with an incomplete conversion group",
                ));
            }
        }
        Ok(written)
    }
}
