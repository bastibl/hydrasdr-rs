use std::cell::RefCell;
use std::collections::VecDeque;
use std::future::Future;
use std::rc::Rc;

use futures_lite::future::block_on;
use hydrasdr_rs::commands::{ReceiverMode, VendorRequest};
use hydrasdr_rs::constants::DEFAULT_BUFFER_SIZE;
use hydrasdr_rs::device::HydraSdr;
use hydrasdr_rs::rfone::{RFONE_RX_ENDPOINT, RFONE_TRANSFER_COUNT};
use hydrasdr_rs::streaming::{
    AsyncBulkInBackend, AsyncStreamingBackend, BulkInCompletion, Transfer,
};
use hydrasdr_rs::usb::control::{AsyncControlBackend, ControlBackend, VendorControlRequest};

#[derive(Debug, Default)]
struct FakeAsyncControl {
    requests: RefCell<Vec<VendorControlRequest>>,
    in_responses: RefCell<Vec<Vec<u8>>>,
}

impl FakeAsyncControl {
    fn with_in_responses(responses: Vec<Vec<u8>>) -> Self {
        Self {
            requests: RefCell::new(Vec::new()),
            in_responses: RefCell::new(responses),
        }
    }

    fn record_control_in(&self, request: VendorControlRequest) -> hydrasdr_rs::Result<Vec<u8>> {
        self.requests.borrow_mut().push(request);
        if self.in_responses.borrow().is_empty() {
            return Ok(vec![1]);
        }
        Ok(self.in_responses.borrow_mut().remove(0))
    }

    fn record_control_out(&self, request: VendorControlRequest) -> hydrasdr_rs::Result<()> {
        self.requests.borrow_mut().push(request);
        Ok(())
    }
}

impl ControlBackend for FakeAsyncControl {
    fn control_in(&self, request: VendorControlRequest) -> hydrasdr_rs::Result<Vec<u8>> {
        self.record_control_in(request)
    }

    fn control_out(&self, request: VendorControlRequest) -> hydrasdr_rs::Result<()> {
        self.record_control_out(request)
    }
}

impl AsyncControlBackend for FakeAsyncControl {
    fn control_in_async(
        &self,
        request: VendorControlRequest,
    ) -> impl Future<Output = hydrasdr_rs::Result<Vec<u8>>> + '_ {
        async move { self.record_control_in(request) }
    }

    fn control_out_async(
        &self,
        request: VendorControlRequest,
    ) -> impl Future<Output = hydrasdr_rs::Result<()>> + '_ {
        async move { self.record_control_out(request) }
    }
}

#[test]
fn async_control_helpers_share_sync_request_encoding_and_update_state() {
    block_on(async {
        let control = FakeAsyncControl::with_in_responses(vec![
            2u32.to_le_bytes().to_vec(),
            [10_000_000u32.to_le_bytes(), 20_000_000u32.to_le_bytes()].concat(),
            vec![1],
            vec![1],
        ]);
        let mut dev = HydraSdr::from_control(control);

        assert_eq!(
            dev.get_samplerates_async().await.unwrap(),
            vec![10_000_000, 20_000_000]
        );
        dev.set_samplerate_async(20_000_000).await.unwrap();
        dev.set_freq_async(915_000_000).await.unwrap();
        dev.set_packing_async(1).await.unwrap();

        let requests = dev.control().requests.borrow();
        assert_eq!(
            requests[0],
            VendorControlRequest::get_samplerates_count(false)
        );
        assert_eq!(requests[1], VendorControlRequest::get_samplerates(2, false));
        assert_eq!(requests[2], VendorControlRequest::set_samplerate(1, 1));
        assert_eq!(
            requests[3],
            VendorControlRequest::set_frequency(915_000_000)
        );
        assert_eq!(requests[4], VendorControlRequest::set_packing(1));
    });
}

#[derive(Debug, Default)]
struct FakeAsyncState {
    control_requests: Vec<VendorControlRequest>,
    opened_endpoints: Vec<u8>,
    async_cleared_halts: Vec<u8>,
    allocated_sizes: Vec<usize>,
    submitted_count: usize,
    pending_count: usize,
    async_next_count: usize,
    cancelled: bool,
    completions: VecDeque<BulkInCompletion<Vec<u8>>>,
}

#[derive(Clone, Debug, Default)]
struct FakeAsyncDevice {
    state: Rc<RefCell<FakeAsyncState>>,
}

impl FakeAsyncDevice {
    fn with_completions(completions: impl IntoIterator<Item = Vec<u8>>) -> Self {
        let this = Self::default();
        this.state.borrow_mut().completions = completions
            .into_iter()
            .map(|buffer| BulkInCompletion {
                actual_len: buffer.len(),
                buffer,
                status: Ok(()),
            })
            .collect();
        this
    }
}

impl ControlBackend for FakeAsyncDevice {
    fn control_in(&self, request: VendorControlRequest) -> hydrasdr_rs::Result<Vec<u8>> {
        self.state.borrow_mut().control_requests.push(request);
        Ok(vec![1])
    }

    fn control_out(&self, request: VendorControlRequest) -> hydrasdr_rs::Result<()> {
        self.state.borrow_mut().control_requests.push(request);
        Ok(())
    }
}

impl AsyncControlBackend for FakeAsyncDevice {
    fn control_in_async(
        &self,
        request: VendorControlRequest,
    ) -> impl Future<Output = hydrasdr_rs::Result<Vec<u8>>> + '_ {
        async move {
            self.state.borrow_mut().control_requests.push(request);
            Ok(vec![1])
        }
    }

    fn control_out_async(
        &self,
        request: VendorControlRequest,
    ) -> impl Future<Output = hydrasdr_rs::Result<()>> + '_ {
        async move {
            self.state.borrow_mut().control_requests.push(request);
            Ok(())
        }
    }
}

impl AsyncStreamingBackend for FakeAsyncDevice {
    type BulkIn = FakeAsyncBulkIn;

    fn bulk_in_async(
        &self,
        endpoint: u8,
    ) -> impl Future<Output = hydrasdr_rs::Result<Self::BulkIn>> + '_ {
        async move {
            self.state.borrow_mut().opened_endpoints.push(endpoint);
            Ok(FakeAsyncBulkIn {
                endpoint,
                state: self.state.clone(),
            })
        }
    }
}

#[derive(Debug)]
struct FakeAsyncBulkIn {
    endpoint: u8,
    state: Rc<RefCell<FakeAsyncState>>,
}

impl AsyncBulkInBackend for FakeAsyncBulkIn {
    type Buffer = Vec<u8>;

    fn clear_halt_async(&mut self) -> impl Future<Output = hydrasdr_rs::Result<()>> + '_ {
        async move {
            self.state
                .borrow_mut()
                .async_cleared_halts
                .push(self.endpoint);
            Ok(())
        }
    }

    fn allocate(&self, len: usize) -> Self::Buffer {
        self.state.borrow_mut().allocated_sizes.push(len);
        vec![0; len]
    }

    fn submit(&mut self, buffer: Self::Buffer) {
        let mut state = self.state.borrow_mut();
        state.submitted_count += 1;
        state.pending_count += 1;
        drop(buffer);
    }

    fn pending(&self) -> usize {
        self.state.borrow().pending_count
    }

    fn next_complete_async(&mut self) -> impl Future<Output = BulkInCompletion<Self::Buffer>> + '_ {
        async move {
            let mut state = self.state.borrow_mut();
            state.async_next_count += 1;
            let completion = state
                .completions
                .pop_front()
                .expect("test queued enough completions");
            state.pending_count -= 1;
            completion
        }
    }

    fn cancel_all(&mut self) {
        self.state.borrow_mut().cancelled = true;
    }
}

#[test]
fn async_streaming_awaits_completions_without_using_sync_wait_path() {
    block_on(async {
        let backend = FakeAsyncDevice::with_completions([
            vec![0x11; DEFAULT_BUFFER_SIZE],
            vec![0x22; DEFAULT_BUFFER_SIZE],
        ]);
        let state = backend.state.clone();
        let mut dev = HydraSdr::from_control(backend);

        let mut callback_count = 0;
        let stats = dev
            .start_rx_async(|transfer: &Transfer<'_>| {
                callback_count += 1;
                assert_eq!(
                    transfer.samples[0],
                    if callback_count == 1 { 0x11 } else { 0x22 }
                );
                assert_eq!(transfer.sample_count, (DEFAULT_BUFFER_SIZE / 2) as i32);
                i32::from(callback_count == 2)
            })
            .await
            .unwrap();

        let state = state.borrow();
        let receiver_modes: Vec<_> = state
            .control_requests
            .iter()
            .filter(|request| request.request == VendorRequest::ReceiverMode)
            .map(|request| request.value)
            .collect();
        assert_eq!(
            receiver_modes,
            vec![
                ReceiverMode::Off as u16,
                ReceiverMode::Rx as u16,
                ReceiverMode::Off as u16,
            ]
        );
        assert_eq!(state.opened_endpoints, vec![RFONE_RX_ENDPOINT]);
        assert_eq!(state.async_cleared_halts, vec![RFONE_RX_ENDPOINT]);
        assert_eq!(state.async_next_count, 2);
        assert_eq!(state.allocated_sizes.len(), RFONE_TRANSFER_COUNT as usize);
        assert!(
            state
                .allocated_sizes
                .iter()
                .all(|size| *size == DEFAULT_BUFFER_SIZE)
        );
        assert!(state.cancelled);
        assert_eq!(stats.buffers_processed, 2);
    });
}
