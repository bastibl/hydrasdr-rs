use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;
use std::time::Duration;

use crate::commands::{ReceiverMode, VendorRequest};
use crate::constants::{DEFAULT_BUFFER_SIZE, PACKED_BUFFER_SIZE};
use crate::device::HydraSdr;
use crate::errors::StatusCode;
use crate::rfone::{RFONE_RX_ENDPOINT, RFONE_TRANSFER_COUNT};
use crate::streaming::{BulkInBackend, BulkInCompletion, StreamingBackend, Transfer};
use crate::types::SampleType;
use crate::usb::control::{ControlBackend, VendorControlRequest};

#[derive(Debug, Default)]
struct FakeState {
    control_requests: Vec<VendorControlRequest>,
    opened_endpoints: Vec<u8>,
    cleared_halts: Vec<u8>,
    allocated_sizes: Vec<usize>,
    submitted_count: usize,
    pending_count: usize,
    wait_count: usize,
    cancelled: bool,
    cancel_count: usize,
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
    fn control_in(&self, request: VendorControlRequest) -> crate::Result<Vec<u8>> {
        self.state.borrow_mut().control_requests.push(request);
        Ok(vec![1])
    }

    fn control_out(&self, request: VendorControlRequest) -> crate::Result<()> {
        self.state.borrow_mut().control_requests.push(request);
        Ok(())
    }
}

impl StreamingBackend for FakeDevice {
    type BulkIn = FakeBulkIn;

    fn bulk_in(&self, endpoint: u8) -> crate::Result<Self::BulkIn> {
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

    fn clear_halt(&mut self) -> crate::Result<()> {
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
        state.wait_count += 1;
        let completion = state.completions.pop_front()?;
        state.pending_count -= 1;
        Some(completion)
    }

    fn cancel_all(&mut self) {
        let mut state = self.state.borrow_mut();
        state.cancelled = true;
        state.cancel_count += 1;
    }
}

fn adc_buffer(seed: usize) -> Vec<u8> {
    let mut raw = vec![0; DEFAULT_BUFFER_SIZE];
    for (i, chunk) in raw.chunks_exact_mut(2).enumerate() {
        let sample = (((i + seed) * 73) % 4096) as u16;
        chunk.copy_from_slice(&sample.to_le_bytes());
    }
    raw
}

fn receiver_modes(state: &FakeState) -> Vec<u16> {
    state
        .control_requests
        .iter()
        .filter(|request| request.request == VendorRequest::ReceiverMode)
        .map(|request| request.value)
        .collect()
}

#[test]
fn start_rx_uses_c_style_receiver_modes_endpoint_and_callback_loop() {
    let first = vec![0x11; DEFAULT_BUFFER_SIZE];
    let second = vec![0x22; DEFAULT_BUFFER_SIZE];
    let backend = FakeDevice::with_completions([first, second]);
    let state = backend.state.clone();
    let mut dev = HydraSdr::from_control(backend);
    dev.set_sample_type(SampleType::Raw).unwrap();

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
    assert_eq!(
        receiver_modes(&state),
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
    dev.set_sample_type(SampleType::Raw).unwrap();

    let mut seen = Vec::new();
    let stats = dev
        .start_rx(|transfer: &Transfer<'_>| {
            seen.push(transfer.samples[0]);
            1
        })
        .unwrap();

    assert_eq!(seen, vec![0x01]);
    assert_eq!(stats.buffers_received, 1);
    assert_eq!(
        state.borrow().submitted_count,
        RFONE_TRANSFER_COUNT as usize
    );
    assert_eq!(state.borrow().completions.len(), 2);
    assert!(!dev.is_streaming());
}

#[test]
fn packed_streaming_uses_c_buffer_size_and_sample_count() {
    let backend = FakeDevice::with_completions([vec![0x5a; PACKED_BUFFER_SIZE]]);
    let state = backend.state.clone();
    let mut dev = HydraSdr::from_control(backend);
    dev.set_sample_type(SampleType::Raw).unwrap();
    dev.set_packing(1).unwrap();

    let mut observed_sample_count = 0;
    let stats = dev
        .start_rx(|transfer: &Transfer<'_>| {
            observed_sample_count = transfer.sample_count;
            assert_eq!(transfer.samples.len(), PACKED_BUFFER_SIZE);
            1
        })
        .unwrap();

    assert_eq!(
        observed_sample_count,
        (((PACKED_BUFFER_SIZE / 2) * 4) / 3) as i32
    );
    assert_eq!(stats.buffers_processed, 1);
    assert!(
        state
            .borrow()
            .allocated_sizes
            .iter()
            .all(|size| *size == PACKED_BUFFER_SIZE)
    );
}

#[test]
fn callback_float32_iq_streaming_keeps_raw_usb_transfer_contract() {
    let raw = adc_buffer(0);
    let backend = FakeDevice::with_completions([raw]);
    let mut dev = HydraSdr::from_control(backend);
    dev.set_sample_type(SampleType::Float32Iq).unwrap();

    let mut observed = None;
    let stats = dev
        .start_rx(|transfer: &Transfer<'_>| {
            observed = Some((
                transfer.samples.len(),
                transfer.sample_count,
                transfer.samples[0],
                transfer.samples[1],
                transfer.dropped_samples,
            ));
            1
        })
        .unwrap();

    let (len, sample_count, first, second, dropped) = observed.unwrap();
    assert_eq!(sample_count, (DEFAULT_BUFFER_SIZE / 2) as i32);
    assert_eq!(len, DEFAULT_BUFFER_SIZE);
    assert_eq!(first, 0);
    assert_eq!(second, 0);
    assert_eq!(dropped, 0);
    assert_eq!(stats.buffers_processed, 1);
}

#[test]
fn direct_rx_stream_reuses_receiver_and_transfers_across_reads() {
    let backend = FakeDevice::with_completions([adc_buffer(0), adc_buffer(10_000)]);
    let state = backend.state.clone();
    let mut dev = HydraSdr::from_control(backend);
    dev.set_sample_type(SampleType::Float32Iq).unwrap();

    let mut stream = dev.start_rx_stream().unwrap();
    {
        let state = state.borrow();
        assert_eq!(
            receiver_modes(&state),
            vec![ReceiverMode::Off as u16, ReceiverMode::Rx as u16]
        );
        assert_eq!(state.opened_endpoints, vec![RFONE_RX_ENDPOINT]);
        assert_eq!(state.cleared_halts, vec![RFONE_RX_ENDPOINT]);
        assert_eq!(state.allocated_sizes.len(), RFONE_TRANSFER_COUNT as usize);
        assert!(!state.cancelled);
    }

    let samples_per_completion = DEFAULT_BUFFER_SIZE / 4;
    let mut first = vec![(0.0, 0.0); samples_per_completion];
    let mut second = vec![(0.0, 0.0); samples_per_completion];
    assert_eq!(
        stream
            .read_float32_iq(&mut first, Duration::from_millis(1))
            .unwrap(),
        samples_per_completion
    );
    assert_eq!(
        stream
            .read_float32_iq(&mut second, Duration::from_millis(1))
            .unwrap(),
        samples_per_completion
    );
    assert!(first.iter().all(|(i, q)| i.is_finite() && q.is_finite()));
    assert!(second.iter().all(|(i, q)| i.is_finite() && q.is_finite()));

    {
        let state = state.borrow();
        assert_eq!(
            receiver_modes(&state),
            vec![ReceiverMode::Off as u16, ReceiverMode::Rx as u16]
        );
        assert_eq!(state.opened_endpoints, vec![RFONE_RX_ENDPOINT]);
        assert_eq!(state.cleared_halts, vec![RFONE_RX_ENDPOINT]);
        assert!(!state.cancelled);
    }

    let stats = dev.stop_rx_stream(stream).unwrap();

    let state = state.borrow();
    assert_eq!(
        receiver_modes(&state),
        vec![
            ReceiverMode::Off as u16,
            ReceiverMode::Rx as u16,
            ReceiverMode::Off as u16,
        ]
    );
    assert!(state.cancelled);
    assert_eq!(state.cancel_count, 1);
    assert_eq!(stats.buffers_received, 2);
    assert_eq!(stats.buffers_processed, 2);
    assert_eq!(stats.buffers_dropped, 0);
}

#[test]
fn direct_rx_stream_converter_state_is_continuous_across_reads() {
    let samples_per_completion = DEFAULT_BUFFER_SIZE / 4;

    let one_shot_backend = FakeDevice::with_completions([adc_buffer(0), adc_buffer(10_000)]);
    let mut one_shot_dev = HydraSdr::from_control(one_shot_backend);
    one_shot_dev.set_sample_type(SampleType::Float32Iq).unwrap();
    let mut one_shot_stream = one_shot_dev.start_rx_stream().unwrap();
    let mut expected = vec![(0.0, 0.0); samples_per_completion * 2];
    assert_eq!(
        one_shot_stream
            .read_float32_iq(&mut expected, Duration::from_millis(1))
            .unwrap(),
        expected.len()
    );
    one_shot_dev.stop_rx_stream(one_shot_stream).unwrap();

    let split_backend = FakeDevice::with_completions([adc_buffer(0), adc_buffer(10_000)]);
    let mut split_dev = HydraSdr::from_control(split_backend);
    split_dev.set_sample_type(SampleType::Float32Iq).unwrap();
    let mut split_stream = split_dev.start_rx_stream().unwrap();
    let mut actual = vec![(0.0, 0.0); samples_per_completion * 2];
    assert_eq!(
        split_stream
            .read_float32_iq(
                &mut actual[..samples_per_completion],
                Duration::from_millis(1)
            )
            .unwrap(),
        samples_per_completion
    );
    assert_eq!(
        split_stream
            .read_float32_iq(
                &mut actual[samples_per_completion..],
                Duration::from_millis(1)
            )
            .unwrap(),
        samples_per_completion
    );
    split_dev.stop_rx_stream(split_stream).unwrap();

    assert_eq!(actual, expected);
}

#[test]
fn direct_rx_stream_serves_leftovers_before_waiting_for_usb() {
    let backend = FakeDevice::with_completions([adc_buffer(0)]);
    let state = backend.state.clone();
    let mut dev = HydraSdr::from_control(backend);
    dev.set_sample_type(SampleType::Float32Iq).unwrap();
    let mut stream = dev.start_rx_stream().unwrap();

    let mut first = [(0.0, 0.0); 1];
    let mut second = [(0.0, 0.0); 1];
    assert_eq!(
        stream
            .read_float32_iq(&mut first, Duration::from_millis(1))
            .unwrap(),
        1
    );
    let waits_after_first = state.borrow().wait_count;
    assert_eq!(
        stream
            .read_float32_iq(&mut second, Duration::from_millis(1))
            .unwrap(),
        1
    );

    assert_eq!(state.borrow().wait_count, waits_after_first);
    dev.stop_rx_stream(stream).unwrap();
}

#[test]
fn direct_rx_stream_timeout_without_completion_returns_zero() {
    let backend = FakeDevice::default();
    let state = backend.state.clone();
    let mut dev = HydraSdr::from_control(backend);
    dev.set_sample_type(SampleType::Float32Iq).unwrap();
    let mut stream = dev.start_rx_stream().unwrap();

    let mut out = [(0.0, 0.0); 4];
    assert_eq!(
        stream
            .read_float32_iq(&mut out, Duration::from_millis(1))
            .unwrap(),
        0
    );
    assert_eq!(state.borrow().wait_count, 1);
    let stats = dev.stop_rx_stream(stream).unwrap();
    assert_eq!(stats, Default::default());
}

#[test]
fn packed_float32_iq_direct_rx_stream_start_is_explicitly_unsupported() {
    let backend = FakeDevice::default();
    let state = backend.state.clone();
    let mut dev = HydraSdr::from_control(backend);
    dev.set_sample_type(SampleType::Float32Iq).unwrap();
    dev.set_packing(1).unwrap();

    let err = dev.start_rx_stream().unwrap_err();

    assert_eq!(err.status_code(), StatusCode::Unsupported);
    assert!(receiver_modes(&state.borrow()).is_empty());
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
    let state = backend.state.clone();
    let mut dev = HydraSdr::from_control(backend);

    dev.stop_rx().unwrap();
    dev.stop_rx().unwrap();

    assert!(!dev.is_streaming());
    assert_eq!(dev.streaming_stats(), Default::default());
    assert_eq!(
        receiver_modes(&state.borrow()),
        vec![ReceiverMode::Off as u16, ReceiverMode::Off as u16]
    );
}
