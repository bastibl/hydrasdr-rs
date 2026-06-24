use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;

use crate::commands::{ReceiverMode, VendorRequest};
use crate::constants::DEFAULT_BUFFER_SIZE;
use crate::device::HydraSdr;
use crate::high_level::DeviceInner as Device;
use crate::rfone::{RFONE_RX_ENDPOINT, RFONE_TRANSFER_COUNT};
use crate::streaming::{AsyncBulkInBackend, AsyncStreamingBackend, BulkInCompletion, Transfer};
use crate::types::{BoardId, SampleType};
use crate::usb::control::{AsyncControlBackend, ControlBackend, VendorControlRequest};
use crate::{Config, SampleFormat};
use futures_lite::future::block_on;

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

    fn record_control_in(&self, request: VendorControlRequest) -> crate::Result<Vec<u8>> {
        self.requests.borrow_mut().push(request);
        if self.in_responses.borrow().is_empty() {
            return Ok(vec![1]);
        }
        Ok(self.in_responses.borrow_mut().remove(0))
    }

    fn record_control_out(&self, request: VendorControlRequest) -> crate::Result<()> {
        self.requests.borrow_mut().push(request);
        Ok(())
    }
}

impl ControlBackend for FakeAsyncControl {
    fn control_in(&self, request: VendorControlRequest) -> crate::Result<Vec<u8>> {
        self.record_control_in(request)
    }

    fn control_out(&self, request: VendorControlRequest) -> crate::Result<()> {
        self.record_control_out(request)
    }
}

impl AsyncControlBackend for FakeAsyncControl {
    async fn control_in_async(&self, request: VendorControlRequest) -> crate::Result<Vec<u8>> {
        self.record_control_in(request)
    }

    async fn control_out_async(&self, request: VendorControlRequest) -> crate::Result<()> {
        self.record_control_out(request)
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
            vec![
                20_000_000, 10_000_000, 5_000_000, 2_500_000, 1_250_000, 625_000, 312_500, 156_250,
            ]
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
struct TrackedAsyncControl {
    sync_requests: RefCell<Vec<VendorControlRequest>>,
    async_requests: RefCell<Vec<VendorControlRequest>>,
    in_responses: RefCell<VecDeque<Vec<u8>>>,
}

impl TrackedAsyncControl {
    fn with_info_responses() -> Self {
        Self {
            sync_requests: RefCell::new(Vec::new()),
            async_requests: RefCell::new(Vec::new()),
            in_responses: RefCell::new(
                [
                    vec![BoardId::HydraSdrRfOneOfficial as u8],
                    b"HydraSDR RFOne async\0".to_vec(),
                    part_id_serial_response(),
                    0x007f_ffffu32.to_le_bytes().to_vec(),
                    0u32.to_le_bytes().to_vec(),
                    0u32.to_le_bytes().to_vec(),
                    0u32.to_le_bytes().to_vec(),
                ]
                .into_iter()
                .collect(),
            ),
        }
    }

    fn next_response(&self, request: VendorControlRequest) -> crate::Result<Vec<u8>> {
        let response_len = request.length;
        if let Some(response) = self.in_responses.borrow_mut().pop_front() {
            Ok(response)
        } else {
            Ok(vec![0; response_len])
        }
    }
}

impl ControlBackend for TrackedAsyncControl {
    fn control_in(&self, request: VendorControlRequest) -> crate::Result<Vec<u8>> {
        self.sync_requests.borrow_mut().push(request.clone());
        self.next_response(request)
    }

    fn control_out(&self, request: VendorControlRequest) -> crate::Result<()> {
        self.sync_requests.borrow_mut().push(request);
        Ok(())
    }
}

impl AsyncControlBackend for TrackedAsyncControl {
    async fn control_in_async(&self, request: VendorControlRequest) -> crate::Result<Vec<u8>> {
        self.async_requests.borrow_mut().push(request.clone());
        self.next_response(request)
    }

    async fn control_out_async(&self, request: VendorControlRequest) -> crate::Result<()> {
        self.async_requests.borrow_mut().push(request);
        Ok(())
    }
}

#[test]
fn async_from_direct_queries_metadata_without_sync_control_calls() {
    block_on(async {
        let control = TrackedAsyncControl::with_info_responses();
        let direct = HydraSdr::from_control(control);
        let device = Device::from_direct_async(direct).await.unwrap();

        assert_eq!(device.info().firmware_version, "HydraSDR RFOne async");
        assert_eq!(device.info().features, 0x007f_ffff);
        assert!(device.direct().control().sync_requests.borrow().is_empty());

        let async_requests = device.direct().control().async_requests.borrow();
        let request_ids: Vec<_> = async_requests
            .iter()
            .map(|request| request.request)
            .collect();
        assert_eq!(
            request_ids,
            vec![
                VendorRequest::BoardIdRead,
                VendorRequest::VersionStringRead,
                VendorRequest::BoardPartIdSerialNoRead,
                VendorRequest::GetCapabilities,
                VendorRequest::GetCapabilities,
                VendorRequest::GetCapabilities,
                VendorRequest::GetCapabilities,
            ]
        );
    });
}

#[derive(Debug, Default)]
struct FakeAsyncState {
    control_requests: Vec<VendorControlRequest>,
    in_responses: VecDeque<Vec<u8>>,
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

    fn with_completions_and_in_responses(
        completions: impl IntoIterator<Item = Vec<u8>>,
        in_responses: impl IntoIterator<Item = Vec<u8>>,
    ) -> Self {
        let this = Self::with_completions(completions);
        this.state.borrow_mut().in_responses = in_responses.into_iter().collect();
        this
    }

    fn for_high_level_raw_stream(completions: impl IntoIterator<Item = Vec<u8>>) -> Self {
        Self::with_completions_and_in_responses(
            completions,
            [
                1u32.to_le_bytes().to_vec(),
                10_000_000u32.to_le_bytes().to_vec(),
                vec![1],
                vec![1],
            ],
        )
    }

    fn record_control_in(&self, request: VendorControlRequest) -> crate::Result<Vec<u8>> {
        let response_len = request.length;
        let mut state = self.state.borrow_mut();
        state.control_requests.push(request);
        if let Some(response) = state.in_responses.pop_front() {
            Ok(response)
        } else {
            let mut response = vec![0; response_len];
            if let Some(first) = response.first_mut() {
                *first = 1;
            }
            Ok(response)
        }
    }
}

fn part_id_serial_response() -> Vec<u8> {
    [
        0x1111_1111u32,
        0x2222_2222,
        0x3333_3333,
        0x4444_4444,
        0x5555_5555,
        0x6666_6666,
    ]
    .into_iter()
    .flat_map(u32::to_le_bytes)
    .collect()
}

impl ControlBackend for FakeAsyncDevice {
    fn control_in(&self, request: VendorControlRequest) -> crate::Result<Vec<u8>> {
        self.record_control_in(request)
    }

    fn control_out(&self, request: VendorControlRequest) -> crate::Result<()> {
        self.state.borrow_mut().control_requests.push(request);
        Ok(())
    }
}

impl AsyncControlBackend for FakeAsyncDevice {
    async fn control_in_async(&self, request: VendorControlRequest) -> crate::Result<Vec<u8>> {
        self.record_control_in(request)
    }

    async fn control_out_async(&self, request: VendorControlRequest) -> crate::Result<()> {
        self.state.borrow_mut().control_requests.push(request);
        Ok(())
    }
}

impl AsyncStreamingBackend for FakeAsyncDevice {
    type BulkIn = FakeAsyncBulkIn;

    async fn bulk_in_async(&self, endpoint: u8) -> crate::Result<Self::BulkIn> {
        self.state.borrow_mut().opened_endpoints.push(endpoint);
        Ok(FakeAsyncBulkIn {
            endpoint,
            state: self.state.clone(),
        })
    }
}

#[derive(Debug)]
struct FakeAsyncBulkIn {
    endpoint: u8,
    state: Rc<RefCell<FakeAsyncState>>,
}

impl AsyncBulkInBackend for FakeAsyncBulkIn {
    type Buffer = Vec<u8>;

    async fn clear_halt_async(&mut self) -> crate::Result<()> {
        self.state
            .borrow_mut()
            .async_cleared_halts
            .push(self.endpoint);
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

    async fn next_complete_async(&mut self) -> BulkInCompletion<Self::Buffer> {
        let mut state = self.state.borrow_mut();
        state.async_next_count += 1;
        let completion = state
            .completions
            .pop_front()
            .expect("test queued enough completions");
        state.pending_count -= 1;
        completion
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
        dev.set_sample_type(SampleType::Raw).unwrap();

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

#[test]
fn async_high_level_stream_drop_turns_receiver_off() {
    block_on(async {
        let backend = FakeAsyncDevice::for_high_level_raw_stream([vec![0x11; DEFAULT_BUFFER_SIZE]]);
        let state = backend.state.clone();
        let direct = HydraSdr::from_control(backend);
        let mut device = Device::from_direct_without_info(direct);
        device
            .configure_async(
                &Config::builder()
                    .sample_format(SampleFormat::RawAdc)
                    .build()
                    .unwrap(),
            )
            .await
            .unwrap();
        state.borrow_mut().control_requests.clear();

        {
            let _stream = device.raw_rx_stream_async().await.unwrap();
        }

        let state = state.borrow();
        assert!(state.cancelled);
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
    });
}

#[test]
fn async_high_level_stream_finish_does_not_duplicate_receiver_off_on_drop() {
    block_on(async {
        let backend = FakeAsyncDevice::for_high_level_raw_stream([vec![0x11; DEFAULT_BUFFER_SIZE]]);
        let state = backend.state.clone();
        let direct = HydraSdr::from_control(backend);
        let mut device = Device::from_direct_without_info(direct);
        device
            .configure_async(
                &Config::builder()
                    .sample_format(SampleFormat::RawAdc)
                    .build()
                    .unwrap(),
            )
            .await
            .unwrap();
        state.borrow_mut().control_requests.clear();

        let stream = device.raw_rx_stream_async().await.unwrap();
        let stats = stream.finish().await.unwrap();

        assert_eq!(stats.buffers_processed, 0);
        let state = state.borrow();
        assert!(state.cancelled);
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
    });
}
