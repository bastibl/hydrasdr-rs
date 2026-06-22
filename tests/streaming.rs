use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;
use std::time::Duration;

use hydrasdr_rs::commands::{ReceiverMode, VendorRequest};
use hydrasdr_rs::constants::DEFAULT_BUFFER_SIZE;
use hydrasdr_rs::device::HydraSdr;
use hydrasdr_rs::errors::StatusCode;
use hydrasdr_rs::rfone::{RFONE_RX_ENDPOINT, RFONE_TRANSFER_COUNT};
use hydrasdr_rs::streaming::{BulkInBackend, BulkInCompletion, StreamingBackend, Transfer};
use hydrasdr_rs::usb::control::{ControlBackend, VendorControlRequest};

#[derive(Debug, Default)]
struct FakeState {
    control_requests: Vec<VendorControlRequest>,
    opened_endpoints: Vec<u8>,
    cleared_halts: Vec<u8>,
    allocated_sizes: Vec<usize>,
    submitted_count: usize,
    pending_count: usize,
    cancelled: bool,
    completions: VecDeque<BulkInCompletion<Vec<u8>>>,
}

#[derive(Clone, Debug, Default)]
struct FakeDevice {
    state: Rc<RefCell<FakeState>>,
}

impl FakeDevice {
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

impl ControlBackend for FakeDevice {
    fn control_in(&self, request: VendorControlRequest) -> hydrasdr_rs::Result<Vec<u8>> {
        self.state.borrow_mut().control_requests.push(request);
        Ok(vec![1])
    }

    fn control_out(&self, request: VendorControlRequest) -> hydrasdr_rs::Result<()> {
        self.state.borrow_mut().control_requests.push(request);
        Ok(())
    }
}

impl StreamingBackend for FakeDevice {
    type BulkIn = FakeBulkIn;

    fn bulk_in(&self, endpoint: u8) -> hydrasdr_rs::Result<Self::BulkIn> {
        self.state.borrow_mut().opened_endpoints.push(endpoint);
        Ok(FakeBulkIn {
            endpoint,
            state: self.state.clone(),
        })
    }
}

#[derive(Debug)]
struct FakeBulkIn {
    endpoint: u8,
    state: Rc<RefCell<FakeState>>,
}

impl BulkInBackend for FakeBulkIn {
    type Buffer = Vec<u8>;

    fn clear_halt(&mut self) -> hydrasdr_rs::Result<()> {
        self.state.borrow_mut().cleared_halts.push(self.endpoint);
        Ok(())
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

    fn wait_next_complete(&mut self, _timeout: Duration) -> Option<BulkInCompletion<Self::Buffer>> {
        let mut state = self.state.borrow_mut();
        let completion = state.completions.pop_front()?;
        state.pending_count -= 1;
        Some(completion)
    }

    fn cancel_all(&mut self) {
        self.state.borrow_mut().cancelled = true;
    }
}

#[test]
fn start_rx_uses_c_style_receiver_modes_endpoint_and_callback_loop() {
    let first = vec![0x11; DEFAULT_BUFFER_SIZE];
    let second = vec![0x22; DEFAULT_BUFFER_SIZE];
    let backend = FakeDevice::with_completions([first, second]);
    let state = backend.state.clone();
    let mut dev = HydraSdr::from_control(backend);

    let mut callback_count = 0;
    let stats = dev
        .start_rx(|transfer: &Transfer<'_>| {
            callback_count += 1;
            assert_eq!(
                transfer.samples[0],
                if callback_count == 1 { 0x11 } else { 0x22 }
            );
            assert_eq!(transfer.sample_count, (DEFAULT_BUFFER_SIZE / 2) as i32);
            assert_eq!(transfer.dropped_samples, 0);
            i32::from(callback_count == 2)
        })
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
            ReceiverMode::Off as u16
        ]
    );
    assert_eq!(state.opened_endpoints, vec![RFONE_RX_ENDPOINT]);
    assert_eq!(state.cleared_halts, vec![RFONE_RX_ENDPOINT]);
    assert_eq!(state.allocated_sizes.len(), RFONE_TRANSFER_COUNT as usize);
    assert!(
        state
            .allocated_sizes
            .iter()
            .all(|size| *size == DEFAULT_BUFFER_SIZE)
    );
    assert!(state.submitted_count >= RFONE_TRANSFER_COUNT as usize);
    assert!(state.cancelled);
    assert_eq!(callback_count, 2);
    assert_eq!(stats.buffers_received, 2);
    assert_eq!(stats.buffers_processed, 2);
    assert_eq!(stats.buffers_dropped, 0);
    assert!(!dev.is_streaming());
}

#[test]
fn callback_can_stop_streaming_before_more_completions_are_processed() {
    let backend = FakeDevice::with_completions([
        vec![0x01; DEFAULT_BUFFER_SIZE],
        vec![0x02; DEFAULT_BUFFER_SIZE],
        vec![0x03; DEFAULT_BUFFER_SIZE],
    ]);
    let state = backend.state.clone();
    let mut dev = HydraSdr::from_control(backend);

    let mut seen = Vec::new();
    let stats = dev
        .start_rx(|transfer: &Transfer<'_>| {
            seen.push(transfer.samples[0]);
            1
        })
        .unwrap();

    assert_eq!(seen, vec![0x01]);
    assert_eq!(stats.buffers_received, 1);
    assert_eq!(state.borrow().completions.len(), 2);
    assert!(!dev.is_streaming());
}

#[test]
fn transfer_error_stops_streaming_reports_libusb_and_cancels_pending() {
    let backend = FakeDevice::default();
    backend
        .state
        .borrow_mut()
        .completions
        .push_back(BulkInCompletion {
            actual_len: 0,
            buffer: vec![0; DEFAULT_BUFFER_SIZE],
            status: Err(StatusCode::LibUsb.into()),
        });
    let state = backend.state.clone();
    let mut dev = HydraSdr::from_control(backend);

    let err = dev.start_rx(|_| 0).unwrap_err();

    assert_eq!(err.status_code(), StatusCode::LibUsb);
    assert!(state.borrow().cancelled);
    assert!(!dev.is_streaming());
}

#[test]
fn stop_rx_is_idempotent_when_streaming_is_idle() {
    let backend = FakeDevice::default();
    let mut dev = HydraSdr::from_control(backend);

    dev.stop_rx().unwrap();
    dev.stop_rx().unwrap();

    assert!(!dev.is_streaming());
    assert_eq!(dev.streaming_stats(), Default::default());
}
