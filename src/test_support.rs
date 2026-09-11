//! Scripted transport shared by protocol, lifecycle, and receive tests.
use crate::{
    errors::Result,
    protocol::{InRequest, OutRequest, VendorRequest},
    usb::{BulkInBackend, BulkInCompletion, ControlBackend, Transport},
};
use nusb::MaybeFuture;
use std::{
    collections::VecDeque,
    future::IntoFuture,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicUsize, Ordering},
    },
    task::{Context, Poll},
    time::Duration,
};
#[derive(Debug, Default)]
pub(crate) struct FakeState {
    pub(crate) fail_packing: AtomicBool,
    pub(crate) pause_control_in_at: AtomicUsize,
    pub(crate) control_in_count: AtomicUsize,
    pub(crate) responses: Mutex<VecDeque<Result<Vec<u8>>>>,
    pub(crate) control_in_requests: Mutex<Vec<InRequest>>,
    pub(crate) pause_bulk: AtomicBool,
    pub(crate) control_out_count: AtomicUsize,
    pub(crate) control_out_requests: Mutex<Vec<OutRequest>>,
    pub(crate) fail_control_out: AtomicBool,
    pub(crate) fail_control_out_at: AtomicUsize,
    pub(crate) pause_control_out_at: AtomicUsize,
    pub(crate) pause_clear_halt: AtomicBool,
    pub(crate) fail_bulk_completion: AtomicBool,
    pub(crate) bulk_in_count: AtomicUsize,
    pub(crate) cancel_count: AtomicUsize,
    pub(crate) events: Mutex<Vec<FakeEvent>>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) enum FakeEvent {
    Control(OutRequest),
    QueueDropped,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct FakeTransport {
    pub(crate) state: Arc<FakeState>,
}

impl ControlBackend for FakeTransport {
    fn control_in(&self, request: InRequest) -> impl MaybeFuture<Output = Result<Vec<u8>>> + use<> {
        FakeTransportIn {
            state: Arc::clone(&self.state),
            request,
        }
    }

    fn control_out(&self, request: OutRequest) -> impl MaybeFuture<Output = Result<()>> + use<> {
        FakeTransportOut {
            state: Arc::clone(&self.state),
            request,
        }
    }
}

struct FakeTransportOut {
    pub(crate) state: Arc<FakeState>,
    pub(crate) request: OutRequest,
}

impl std::future::IntoFuture for FakeTransportOut {
    type Output = Result<()>;
    type IntoFuture = std::pin::Pin<Box<dyn std::future::Future<Output = Result<()>> + Send>>;

    fn into_future(self) -> Self::IntoFuture {
        Box::pin(async move {
            self.state
                .events
                .lock()
                .unwrap()
                .push(FakeEvent::Control(self.request.clone()));
            self.state
                .control_out_requests
                .lock()
                .expect("control request lock")
                .push(self.request);
            let call = self.state.control_out_count.fetch_add(1, Ordering::SeqCst) + 1;
            // Tests explicitly poll again after clearing the pause.
            std::future::poll_fn(|_| {
                if self.state.pause_control_out_at.load(Ordering::SeqCst) == call {
                    Poll::Pending
                } else {
                    Poll::Ready(())
                }
            })
            .await;
            if self.state.fail_control_out.load(Ordering::SeqCst)
                || self.state.fail_control_out_at.load(Ordering::SeqCst) == call
            {
                Err(nusb::transfer::TransferError::Fault.into())
            } else {
                Ok(())
            }
        })
    }
}

impl MaybeFuture for FakeTransportOut {
    #[cfg(not(target_arch = "wasm32"))]
    fn wait(self) -> Result<()> {
        self.state
            .events
            .lock()
            .unwrap()
            .push(FakeEvent::Control(self.request.clone()));
        self.state
            .control_out_requests
            .lock()
            .expect("control request lock")
            .push(self.request);
        let call = self.state.control_out_count.fetch_add(1, Ordering::SeqCst) + 1;
        if self.state.fail_control_out.load(Ordering::SeqCst)
            || self.state.fail_control_out_at.load(Ordering::SeqCst) == call
        {
            Err(nusb::transfer::TransferError::Fault.into())
        } else {
            Ok(())
        }
    }
}

impl Transport for FakeTransport {
    type BulkIn = FakeBulkIn;
    fn bulk_in(&self, _endpoint: u8) -> Result<FakeBulkIn> {
        self.state.bulk_in_count.fetch_add(1, Ordering::SeqCst);
        let mut bulk = FakeBulkIn::default();
        bulk.state = Arc::clone(&self.state);
        Ok(bulk)
    }
}
#[derive(Debug, Default)]
pub(crate) struct FakeBulkIn {
    pub(crate) state: Arc<FakeState>,
    pub(crate) submitted: VecDeque<Vec<u8>>,
    pub(crate) submit_count: usize,
    pub(crate) pending_once: bool,
    pub(crate) fail_next: bool,
    pub(crate) short_next: bool,
    pub(crate) timeout_next: bool,
    pub(crate) last_timeout: Option<Duration>,
    pub(crate) cancelled: bool,
}
impl Drop for FakeBulkIn {
    fn drop(&mut self) {
        self.state
            .events
            .lock()
            .unwrap()
            .push(FakeEvent::QueueDropped);
    }
}
impl FakeBulkIn {
    fn complete(&mut self) -> BulkInCompletion<Vec<u8>> {
        let buffer = self
            .submitted
            .pop_front()
            .expect("fake endpoint has submitted buffers");
        let status = if std::mem::take(&mut self.fail_next)
            || self
                .state
                .fail_bulk_completion
                .swap(false, Ordering::SeqCst)
        {
            Err(nusb::transfer::TransferError::Fault.into())
        } else {
            Ok(())
        };
        let actual_len = buffer.len()
            - if std::mem::take(&mut self.short_next) {
                2
            } else {
                0
            };
        BulkInCompletion {
            buffer,
            actual_len,
            status,
        }
    }
}
impl BulkInBackend for FakeBulkIn {
    type Buffer = Vec<u8>;
    fn clear_halt(&mut self) -> impl MaybeFuture<Output = Result<()>> + use<> {
        ClearHalt(Arc::clone(&self.state))
    }
    fn allocate(&self, len: usize) -> Vec<u8> {
        vec![0; len]
    }
    fn submit(&mut self, buffer: Vec<u8>) {
        self.submit_count += 1;
        self.submitted.push_back(buffer);
    }
    fn pending(&self) -> usize {
        self.submitted.len()
    }
    fn poll_next_complete(&mut self, cx: &mut Context<'_>) -> Poll<BulkInCompletion<Vec<u8>>> {
        if self.state.pause_bulk.load(Ordering::SeqCst) {
            return Poll::Pending;
        }
        if std::mem::take(&mut self.pending_once) {
            cx.waker().wake_by_ref();
            return Poll::Pending;
        }
        Poll::Ready(self.complete())
    }
    fn wait_next_complete(&mut self, timeout: Duration) -> Option<BulkInCompletion<Vec<u8>>> {
        self.last_timeout = Some(timeout);
        if std::mem::take(&mut self.timeout_next) {
            None
        } else {
            Some(self.complete())
        }
    }
    fn cancel_all(&mut self) {
        self.cancelled = true;
        self.state.cancel_count.fetch_add(1, Ordering::SeqCst);
    }
}
struct ClearHalt(Arc<FakeState>);
impl IntoFuture for ClearHalt {
    type Output = Result<()>;
    type IntoFuture = crate::maybe_future::OperationFuture<Self::Output>;
    fn into_future(self) -> Self::IntoFuture {
        Box::pin(async move {
            std::future::poll_fn(|_| {
                if self.0.pause_clear_halt.load(Ordering::SeqCst) {
                    Poll::Pending
                } else {
                    Poll::Ready(Ok(()))
                }
            })
            .await
        })
    }
}
impl MaybeFuture for ClearHalt {
    fn wait(self) -> Result<()> {
        Ok(())
    }
}

struct FakeTransportIn {
    state: Arc<FakeState>,
    request: InRequest,
}
impl FakeTransportIn {
    fn record(&self) -> usize {
        self.state
            .control_in_requests
            .lock()
            .unwrap()
            .push(self.request.clone());
        self.state.control_in_count.fetch_add(1, Ordering::SeqCst) + 1
    }
    fn response(self) -> Result<Vec<u8>> {
        if let Some(response) = self.state.responses.lock().unwrap().pop_front() {
            return response;
        }
        if self.state.fail_packing.load(Ordering::SeqCst)
            && self.request.request == VendorRequest::SetPacking
        {
            return Err(nusb::transfer::TransferError::Fault.into());
        }
        Ok(match self.request.request {
            VendorRequest::VersionStringRead => b"HydraSDR RF test".to_vec(),
            VendorRequest::BoardPartIdSerialNoRead => vec![0; 24],
            _ => vec![1],
        })
    }
}
impl IntoFuture for FakeTransportIn {
    type Output = Result<Vec<u8>>;
    type IntoFuture = crate::maybe_future::OperationFuture<Self::Output>;
    fn into_future(self) -> Self::IntoFuture {
        Box::pin(async move {
            let call = self.record();
            std::future::poll_fn(|_| {
                if self.state.pause_control_in_at.load(Ordering::SeqCst) == call {
                    Poll::Pending
                } else {
                    Poll::Ready(())
                }
            })
            .await;
            self.response()
        })
    }
}
impl MaybeFuture for FakeTransportIn {
    fn wait(self) -> Result<Vec<u8>> {
        self.record();
        self.response()
    }
}
