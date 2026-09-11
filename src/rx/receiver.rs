//! One receiver lifecycle for every sample mode and execution mechanism.

use super::{SampleBlock, StreamingStats, processing::Processor, queue::Queue};
use crate::{
    Complex32,
    config::{F32Iq, RawAdc, SampleMode},
    errors::{Error, Result},
    maybe_future::{Either, MaybeFutureExt, ready},
    protocol::{OutRequest, ReceiverMode},
    rfone::RFONE_RX_ENDPOINT,
    session::{ReceiverState, RxStreamClaim},
    usb::{NusbTransport, Transport},
};
use nusb::MaybeFuture;
use std::{
    future::{Future, IntoFuture},
    pin::Pin,
    sync::Arc,
    task::{Context, Poll},
    time::Duration,
};

pub(crate) struct Receiver<C: Transport = NusbTransport, M: SampleMode = F32Iq> {
    // The endpoint and its buffers must be released before the claim's drop cleanup.
    pub(super) queue: Option<Queue<C::BulkIn>>,
    pub(super) processor: M::Processor,
    pub(super) stats: StreamingStats,
    pub(crate) claim: RxStreamClaim<C>,
}

impl<C: Transport, M: SampleMode> Receiver<C, M> {
    pub(crate) fn new(claim: RxStreamClaim<C>) -> Self {
        Self {
            queue: None,
            processor: Default::default(),
            stats: StreamingStats::default(),
            claim,
        }
    }
    pub(crate) fn current_stats(&self) -> StreamingStats {
        self.queue
            .as_ref()
            .map_or(self.stats, |q| self.stats.combined(q.stats))
    }
    pub(crate) fn retire_stream(&mut self) {
        if let Some(mut queue) = self.queue.take() {
            self.stats.accumulate(queue.close());
        }
        self.processor.reset();
    }
    pub(crate) fn start(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        ready(()).continue_with(move |_| {
            ready(self.claim.begin_start()).and_then(move |start| {
                if start {
                    let desired = self.claim.shared.receive_settings();
                    let fresh = !self.queue.as_ref().is_some_and(|q| {
                        q.initialized && !q.is_closed() && q.settings.packing == desired.packing
                    });
                    Either::left(self.start_pipeline(fresh))
                } else {
                    Either::right(ready(Ok(())))
                }
            })
        })
    }

    fn start_pipeline(&mut self, fresh: bool) -> impl MaybeFuture<Output = Result<()>> + '_ {
        let settings = self.claim.shared.receive_settings();
        if fresh {
            self.retire_stream();
        }
        if let Some(queue) = self.queue.as_mut() {
            if queue.settings.decimation != settings.decimation {
                queue.discard_current();
                self.processor.reset();
            }
            queue.settings = settings;
        }
        let control = Arc::clone(&self.claim.shared.control);
        let fallback = Arc::clone(&control);
        let off = if fresh {
            Either::left(control.control_out(OutRequest::receiver_mode(ReceiverMode::Off)))
        } else {
            Either::right(ready(Ok(())))
        };
        let queue = &mut self.queue;
        let claim = &mut self.claim;
        let run = off.and_then(move |_| {
            let prepare = if fresh {
                control
                    .bulk_in(RFONE_RX_ENDPOINT)
                    .map(|bulk| *queue = Some(Queue::new(bulk, settings)))
            } else {
                Ok(())
            };
            ready(prepare).and_then(move |_| {
                control
                    .control_out(OutRequest::receiver_mode(ReceiverMode::Rx))
                    .and_then(move |_| {
                        if fresh {
                            Either::left(queue.as_mut().expect("prepared queue").initialize())
                        } else {
                            Either::right(ready(Ok(())))
                        }
                    })
            })
        });
        run.continue_with(move |result| match result {
            Ok(()) => {
                claim.set_receiver_state(ReceiverState::Running);
                Either::left(ready(Ok(())))
            }
            Err(error) => Either::right(
                fallback
                    .control_out(OutRequest::receiver_mode(ReceiverMode::Off))
                    .map(move |_| Err(error)),
            ),
        })
    }

    pub(crate) fn stop(&mut self) -> impl MaybeFuture<Output = Result<StreamingStats>> + '_ {
        ready(()).continue_with(move |_| {
            if !self.claim.begin_stop() {
                return Either::right(ready(Ok(self.current_stats())));
            }
            Either::left(
                self.claim
                    .shared
                    .control
                    .control_out(OutRequest::receiver_mode(ReceiverMode::Off))
                    .map(move |result| {
                        result?;
                        if let Some(queue) = &mut self.queue {
                            queue.pause();
                        }
                        self.processor.reset();
                        self.claim.set_receiver_state(ReceiverState::Stopped);
                        Ok(self.current_stats())
                    }),
            )
        })
    }
    fn ensure_running(&self) -> Result<()> {
        if self.claim.receiver_state() == ReceiverState::Running {
            Ok(())
        } else {
            Err(Error::stream_closed(
                "RX stream is stopped or requires cleanup",
            ))
        }
    }
    fn failed_read<T>(&mut self, result: Result<T>) -> Result<T> {
        if result.is_err() {
            self.retire_stream();
            self.claim
                .set_receiver_state(ReceiverState::CleanupRequired);
        }
        result
    }
    pub(crate) fn close(self) -> CloseOperation<C, M> {
        CloseOperation { receiver: self }
    }
    fn into_close_parts(mut self) -> (RxStreamClaim<C>, StreamingStats) {
        self.retire_stream();
        (self.claim, self.stats)
    }
}

impl<C: Transport> Receiver<C, RawAdc> {
    pub(crate) fn packing_changed(&self) -> bool {
        self.queue
            .as_ref()
            .is_some_and(|q| q.settings.packing != self.claim.shared.receive_settings().packing)
    }
    pub(crate) fn refresh_packing(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        ready(self.ensure_running()).and_then(move |_| {
            if self.packing_changed() {
                self.claim
                    .set_receiver_state(ReceiverState::CleanupRequired);
                Either::left(self.start_pipeline(true))
            } else {
                Either::right(ready(Ok(())))
            }
        })
    }
    pub(crate) fn poll_next_block(&mut self, cx: &mut Context<'_>) -> Poll<Result<()>> {
        self.ensure_running()?;
        let queue = self
            .queue
            .as_mut()
            .ok_or(Error::stream_closed("RX queue is closed"))?;
        queue.release_current();
        match queue.poll_fill(cx) {
            Poll::Pending => Poll::Pending,
            Poll::Ready(result) => Poll::Ready(self.failed_read(result)),
        }
    }
    pub(crate) fn current_block(&self) -> Result<SampleBlock<'_>> {
        self.queue
            .as_ref()
            .ok_or(Error::stream_closed("RX queue is closed"))?
            .block()
    }
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn next_block_blocking(
        &mut self,
        timeout: Duration,
    ) -> Result<Option<SampleBlock<'_>>> {
        self.refresh_packing().wait()?;
        let queue = self
            .queue
            .as_mut()
            .ok_or(Error::stream_closed("RX queue is closed"))?;
        queue.release_current();
        let result = queue.wait_fill(timeout);
        if self.failed_read(result)? {
            self.current_block().map(Some)
        } else {
            Ok(None)
        }
    }
    pub(crate) fn next_block(&mut self, timeout: Option<Duration>) -> NextBlockOperation<'_, C> {
        NextBlockOperation {
            receiver: self,
            timeout,
        }
    }
}

impl<C: Transport> Receiver<C, F32Iq> {
    fn begin_read(&mut self) -> Result<()> {
        self.ensure_running()?;
        let settings = self.claim.shared.receive_settings();
        let queue = self
            .queue
            .as_mut()
            .ok_or(Error::stream_closed("RX queue is closed"))?;
        if queue.settings.decimation != settings.decimation {
            queue.discard_current();
            self.processor.reset();
            queue.settings.decimation = settings.decimation;
        }
        Ok(())
    }
    pub(crate) fn poll_read(
        &mut self,
        out: &mut [Complex32],
        cx: &mut Context<'_>,
    ) -> Poll<Result<usize>> {
        self.begin_read()?;
        if out.is_empty() {
            return Poll::Ready(Ok(0));
        }
        let queue = self.queue.as_mut().expect("running queue");
        if queue.current.is_none() && !self.processor.has_pending() {
            match queue.poll_fill(cx) {
                Poll::Pending => return Poll::Pending,
                Poll::Ready(Err(error)) => return Poll::Ready(self.failed_read(Err(error))),
                Poll::Ready(Ok(())) => {}
            }
        }
        let result = self.processor.process(queue, out);
        Poll::Ready(self.failed_read(result))
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn read_blocking(&mut self, out: &mut [Complex32], timeout: Duration) -> Result<usize> {
        self.begin_read()?;
        if out.is_empty() {
            return Ok(0);
        }
        let queue = self.queue.as_mut().expect("running queue");
        if queue.current.is_none() && !self.processor.has_pending() {
            match queue.wait_fill(timeout) {
                Err(error) => return self.failed_read(Err(error)),
                Ok(false) => return Ok(0),
                Ok(true) => {}
            }
        }
        let result = self.processor.process(queue, out);
        self.failed_read(result)
    }
    pub(crate) fn read<'a>(
        &'a mut self,
        out: &'a mut [Complex32],
        timeout: Option<Duration>,
    ) -> ReadOperation<'a, C> {
        ReadOperation {
            receiver: self,
            out,
            timeout,
            completed: false,
        }
    }
}

pub(crate) struct CloseOperation<C: Transport, M: SampleMode> {
    receiver: Receiver<C, M>,
}
impl<C: Transport, M: SampleMode> IntoFuture for CloseOperation<C, M>
where
    M::Processor: 'static,
{
    type Output = Result<StreamingStats>;
    type IntoFuture = crate::maybe_future::OperationFuture<Self::Output>;
    fn into_future(self) -> Self::IntoFuture {
        Box::pin(async move {
            let (mut claim, stats) = self.receiver.into_close_parts();
            claim.close().await?;
            Ok(stats)
        })
    }
}
impl<C: Transport, M: SampleMode> MaybeFuture for CloseOperation<C, M>
where
    M::Processor: 'static,
{
    #[cfg(not(target_arch = "wasm32"))]
    fn wait(self) -> Result<StreamingStats> {
        let (mut claim, stats) = self.receiver.into_close_parts();
        claim.close().wait()?;
        Ok(stats)
    }
}

pub(crate) struct ReadOperation<'a, C: Transport> {
    receiver: &'a mut Receiver<C, F32Iq>,
    out: &'a mut [Complex32],
    timeout: Option<Duration>,
    completed: bool,
}
impl<C: Transport> Future for ReadOperation<'_, C> {
    type Output = Result<usize>;
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = &mut *self;
        let _ = this.timeout;
        assert!(!this.completed, "read future polled after completion");
        let result = this.receiver.poll_read(this.out, cx);
        this.completed = result.is_ready();
        result
    }
}
impl<C: Transport> MaybeFuture for ReadOperation<'_, C> {
    #[cfg(not(target_arch = "wasm32"))]
    fn wait(self) -> Result<usize> {
        self.receiver
            .read_blocking(self.out, self.timeout.unwrap_or(Duration::MAX))
    }
}

pub(crate) struct NextBlockOperation<'a, C: Transport> {
    pub(crate) receiver: &'a mut Receiver<C, RawAdc>,
    pub(crate) timeout: Option<Duration>,
}
#[cfg(not(target_arch = "wasm32"))]
type RestartFuture<'a> = Pin<Box<dyn Future<Output = Result<Option<SampleBlock<'a>>>> + Send + 'a>>;
#[cfg(target_arch = "wasm32")]
type RestartFuture<'a> = Pin<Box<dyn Future<Output = Result<Option<SampleBlock<'a>>>> + 'a>>;
pub(crate) enum NextBlockFuture<'a, C: Transport> {
    Direct(Option<&'a mut Receiver<C, RawAdc>>),
    Restarting(RestartFuture<'a>),
    Transitioning,
}
impl<'a, C: Transport> IntoFuture for NextBlockOperation<'a, C> {
    type Output = Result<Option<SampleBlock<'a>>>;
    type IntoFuture = NextBlockFuture<'a, C>;
    fn into_future(self) -> Self::IntoFuture {
        let _ = self.timeout;
        NextBlockFuture::Direct(Some(self.receiver))
    }
}
impl<'a, C: Transport> Future for NextBlockFuture<'a, C> {
    type Output = Result<Option<SampleBlock<'a>>>;
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.as_mut().get_mut();
        if matches!(this, Self::Direct(Some(receiver)) if receiver.packing_changed()) {
            let Self::Direct(Some(receiver)) = std::mem::replace(this, Self::Transitioning) else {
                unreachable!()
            };
            *this = Self::Restarting(Box::pin(async move {
                receiver.refresh_packing().await?;
                std::future::poll_fn(|cx| receiver.poll_next_block(cx)).await?;
                receiver.current_block().map(Some)
            }));
        }
        match this {
            Self::Direct(receiver) => {
                let receiver = receiver
                    .take()
                    .expect("next-block future polled after completion");
                match receiver.poll_next_block(cx) {
                    Poll::Pending => {
                        *this = Self::Direct(Some(receiver));
                        Poll::Pending
                    }
                    Poll::Ready(Err(error)) => Poll::Ready(Err(error)),
                    Poll::Ready(Ok(())) => {
                        let receiver: &'a Receiver<C, RawAdc> = receiver;
                        Poll::Ready(receiver.current_block().map(Some))
                    }
                }
            }
            Self::Restarting(future) => future.as_mut().poll(cx),
            Self::Transitioning => unreachable!("synchronous state transition"),
        }
    }
}
impl<C: Transport> MaybeFuture for NextBlockOperation<'_, C> {
    #[cfg(not(target_arch = "wasm32"))]
    fn wait(self) -> Self::Output {
        self.receiver
            .next_block_blocking(self.timeout.unwrap_or(Duration::MAX))
    }
}
