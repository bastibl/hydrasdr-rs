use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;
use std::time::Duration;

use crate::commands::{ReceiverMode, RfPort, VendorRequest};
use crate::constants::DEFAULT_BUFFER_SIZE;
use crate::device::HydraSdr;
use crate::errors::{Error, StatusCode};
use crate::high_level::DeviceInner as Device;
use crate::rfone::{RFONE_RX_ENDPOINT, RFONE_TRANSFER_COUNT};
use crate::streaming::{BulkInBackend, BulkInCompletion, StreamingBackend, StreamingStats};
use crate::types::{BoardId, DecimationMode, SampleType};
use crate::usb::control::{ControlBackend, VendorControlRequest};
use crate::{Bandwidth, Config, DeviceSelector, GainConfig, GainPreset, SampleBlock, SampleFormat};

#[derive(Debug, Default)]
struct FakeState {
    control_requests: Vec<VendorControlRequest>,
    in_responses: VecDeque<Vec<u8>>,
    opened_endpoints: Vec<u8>,
    cleared_halts: Vec<u8>,
    allocated_sizes: Vec<usize>,
    submitted_count: usize,
    pending_count: usize,
    cancelled: bool,
    fail_receiver_off_after_rx: bool,
    completions: VecDeque<BulkInCompletion<Vec<u8>>>,
}

#[derive(Clone, Debug, Default)]
struct FakeDevice {
    state: Rc<RefCell<FakeState>>,
}

impl FakeDevice {
    fn with_info_responses() -> Self {
        let this = Self::default();
        let mut state = this.state.borrow_mut();
        state
            .in_responses
            .push_back(vec![BoardId::HydraSdrRfOneOfficial as u8]);
        state
            .in_responses
            .push_back(b"HydraSDR RFOne test\0".to_vec());
        state.in_responses.push_back(part_id_serial_response());
        state
            .in_responses
            .push_back(0x007f_ffffu32.to_le_bytes().to_vec());
        state.in_responses.push_back(vec![0, 0, 0, 0]);
        state.in_responses.push_back(vec![0, 0, 0, 0]);
        state.in_responses.push_back(vec![0, 0, 0, 0]);
        drop(state);
        this
    }

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

    fn with_completions_and_final_receiver_off_error(
        completions: impl IntoIterator<Item = Vec<u8>>,
    ) -> Self {
        let this = Self::with_completions(completions);
        this.state.borrow_mut().fail_receiver_off_after_rx = true;
        this
    }
}

impl ControlBackend for FakeDevice {
    fn control_in(&self, request: VendorControlRequest) -> crate::Result<Vec<u8>> {
        let response_len = request.length;
        self.state.borrow_mut().control_requests.push(request);
        if let Some(response) = self.state.borrow_mut().in_responses.pop_front() {
            Ok(response)
        } else {
            let mut response = vec![0; response_len];
            if let Some(first) = response.first_mut() {
                *first = 1;
            }
            Ok(response)
        }
    }

    fn control_out(&self, request: VendorControlRequest) -> crate::Result<()> {
        let mut state = self.state.borrow_mut();
        let should_fail = state.fail_receiver_off_after_rx
            && request.request == VendorRequest::ReceiverMode
            && request.value == ReceiverMode::Off as u16
            && state.control_requests.iter().any(|previous| {
                previous.request == VendorRequest::ReceiverMode
                    && previous.value == ReceiverMode::Rx as u16
            });
        state.control_requests.push(request);
        if should_fail {
            return Err(Error::Status(StatusCode::LibUsb));
        }
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
        let completion = state.completions.pop_front()?;
        state.pending_count -= 1;
        Some(completion)
    }

    fn cancel_all(&mut self) {
        self.state.borrow_mut().cancelled = true;
    }
}

#[test]
fn config_builder_validates_safe_ranges_before_usb_io() {
    assert_eq!(crate::Device::builder().selector(), DeviceSelector::First);
    assert_eq!(
        crate::Device::builder()
            .serial(0x0123_4567_89ab_cdef)
            .selector(),
        DeviceSelector::Serial(0x0123_4567_89ab_cdef)
    );

    let err = Config::builder().frequency_hz(0).build().unwrap_err();
    assert_eq!(err.status_code(), StatusCode::InvalidParam);
    assert!(err.to_string().contains("frequency_hz"));

    let err = Config::builder().sample_rate_hz(9_999).build().unwrap_err();
    assert_eq!(err.status_code(), StatusCode::InvalidParam);
    assert!(err.to_string().contains("sample_rate_hz"));

    let err = Config::builder()
        .bandwidth(Bandwidth::ManualHz(999))
        .build()
        .unwrap_err();
    assert_eq!(err.status_code(), StatusCode::InvalidParam);
    assert!(err.to_string().contains("bandwidth_hz"));

    let err = Config::builder()
        .sample_format(SampleFormat::RawAdc)
        .decimation_mode(DecimationMode::HighDefinition)
        .build()
        .unwrap_err();
    assert_eq!(err.status_code(), StatusCode::InvalidParam);
    assert!(err.to_string().contains("decimation_mode"));

    let config = Config::builder()
        .frequency_hz(100_000_000)
        .sample_rate_hz(10_000_000)
        .bandwidth(Bandwidth::Auto)
        .sample_format(SampleFormat::F32Iq)
        .decimation_mode(DecimationMode::HighDefinition)
        .rf_port(RfPort::Rx1)
        .gain(GainPreset::Linearity(12))
        .bias_tee(false)
        .packing(true)
        .build()
        .unwrap();

    assert_eq!(config.frequency_hz(), 100_000_000);
    assert_eq!(config.sample_rate_hz(), 10_000_000);
    assert_eq!(config.sample_format().sample_type(), SampleType::Float32Iq);
    assert_eq!(config.decimation_mode(), DecimationMode::HighDefinition);
}

#[test]
fn sample_formats_map_to_implemented_direct_sample_types() {
    assert_eq!(SampleFormat::RawAdc.sample_type(), SampleType::Raw);
    assert_eq!(SampleFormat::F32Iq.sample_type(), SampleType::Float32Iq);
}

#[test]
fn config_apply_uses_direct_api_in_c_documented_order() {
    let control = FakeDevice::default();
    let state = control.state.clone();
    let mut direct = HydraSdr::from_control(control);
    let config = Config::builder()
        .frequency_hz(144_500_000)
        .sample_rate_hz(10_000_000)
        .bandwidth(Bandwidth::ManualHz(5_000_000))
        .sample_format(SampleFormat::RawAdc)
        .rf_port(RfPort::Rx2)
        .gain(GainConfig::Manual {
            lna: Some(8),
            mixer: Some(6),
            vga: Some(4),
            lna_agc: Some(false),
            mixer_agc: None,
        })
        .bias_tee(true)
        .packing(true)
        .build()
        .unwrap();

    config.apply_direct(&mut direct).unwrap();

    let requests: Vec<_> = state
        .borrow()
        .control_requests
        .iter()
        .map(|request| request.request)
        .collect();
    assert_eq!(
        requests,
        vec![
            VendorRequest::SetFreq,
            VendorRequest::GetBandwidths,
            VendorRequest::GetBandwidths,
            VendorRequest::SetBandwidth,
            VendorRequest::GetSamplerates,
            VendorRequest::GetSamplerates,
            VendorRequest::SetSamplerate,
            VendorRequest::SetRfPort,
            VendorRequest::SetLnaGain,
            VendorRequest::SetMixerGain,
            VendorRequest::SetVgaGain,
            VendorRequest::SetLnaAgc,
            VendorRequest::SetRfBiasCmd,
            VendorRequest::SetPacking,
        ]
    );
    assert_eq!(direct.get_sample_type(), SampleType::Raw);
}

#[test]
fn high_level_device_caches_info_and_applies_configuration() {
    let control = FakeDevice::with_info_responses();
    let state = control.state.clone();
    let direct = HydraSdr::from_control(control);
    let mut device = Device::from_direct(direct).unwrap();

    assert_eq!(device.info().board_name, "HydraSDR RFOne");
    assert_eq!(device.direct().get_sample_type(), SampleType::Float32Iq);

    let config = Config::builder()
        .frequency_hz(100_000_000)
        .sample_rate_hz(10_000_000)
        .sample_format(SampleFormat::RawAdc)
        .build()
        .unwrap();
    device.configure(&config).unwrap();
    assert_eq!(device.direct().get_sample_type(), SampleType::Raw);
    assert!(!device.direct().is_streaming());

    assert!(state.borrow().control_requests.len() > 7);
}

#[test]
fn raw_rx_stream_reads_sample_blocks() {
    let first = vec![0x11; DEFAULT_BUFFER_SIZE];
    let second = vec![0x22; DEFAULT_BUFFER_SIZE];
    let control = FakeDevice::with_completions([first, second]);
    let state = control.state.clone();
    let direct = HydraSdr::from_control(control);
    let mut device = Device::from_direct_without_info(direct);
    device
        .configure(
            &Config::builder()
                .frequency_hz(100_000_000)
                .sample_rate_hz(10_000_000)
                .sample_format(SampleFormat::RawAdc)
                .build()
                .unwrap(),
        )
        .unwrap();

    let mut stream = device.raw_rx_stream().unwrap();
    let mut seen = Vec::new();
    {
        let block = stream.next_block().unwrap().unwrap();
        seen.push((
            block.raw_bytes()[0],
            block.sample_count(),
            block.sample_format(),
        ));
    }
    {
        let block = stream.next_block().unwrap().unwrap();
        seen.push((
            block.raw_bytes()[0],
            block.sample_count(),
            block.sample_format(),
        ));
    }
    let stats = stream.finish().unwrap();

    assert_eq!(
        seen,
        vec![
            (0x11, (DEFAULT_BUFFER_SIZE / 2) as i32, SampleFormat::RawAdc),
            (0x22, (DEFAULT_BUFFER_SIZE / 2) as i32, SampleFormat::RawAdc),
        ]
    );
    assert_eq!(stats.buffers_received, 2);
    assert_eq!(stats.buffers_processed, 2);
    assert!(!device.direct().is_streaming());

    let state = state.borrow();
    assert_eq!(state.opened_endpoints, vec![RFONE_RX_ENDPOINT]);
    assert_eq!(state.cleared_halts, vec![RFONE_RX_ENDPOINT]);
    assert_eq!(state.allocated_sizes.len(), RFONE_TRANSFER_COUNT as usize);
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
            ReceiverMode::Off as u16
        ]
    );
}

#[test]
fn raw_rx_stream_rejects_converted_configs() {
    let control = FakeDevice::default();
    let state = control.state.clone();
    let direct = HydraSdr::from_control(control);
    let mut device = Device::from_direct_without_info(direct);

    let err = match device.raw_rx_stream() {
        Ok(_) => panic!("raw stream should reject F32Iq configuration"),
        Err(err) => err,
    };

    assert_eq!(err.status_code(), StatusCode::InvalidParam);
    assert!(state.borrow().control_requests.is_empty());
}

#[test]
fn raw_rx_stream_stop_and_finish_are_idempotent_when_idle() {
    let control = FakeDevice::default();
    let state = control.state.clone();
    let direct = HydraSdr::from_control(control);
    let mut device = Device::from_direct_without_info(direct);
    device
        .configure(
            &Config::builder()
                .sample_format(SampleFormat::RawAdc)
                .build()
                .unwrap(),
        )
        .unwrap();

    let mut stream = device.raw_rx_stream().unwrap();
    stream.stop().unwrap();
    stream.stop().unwrap();
    let stats: StreamingStats = stream.finish().unwrap();

    assert_eq!(stats, StreamingStats::default());
    let receiver_modes: Vec<_> = state
        .borrow()
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
}

#[test]
fn owned_raw_rx_stream_finish_preserves_device_stats_and_error_on_stop_failure() {
    let control = FakeDevice::with_completions_and_final_receiver_off_error([
        vec![0x11; DEFAULT_BUFFER_SIZE],
    ]);
    let state = control.state.clone();
    let direct = HydraSdr::from_control(control);
    let mut device = Device::from_direct_without_info(direct);
    device
        .configure(
            &Config::builder()
                .sample_format(SampleFormat::RawAdc)
                .build()
                .unwrap(),
        )
        .unwrap();
    state.borrow_mut().control_requests.clear();

    let mut stream = device.into_raw_rx_stream_owned().unwrap();
    let block = stream.next_block().unwrap().unwrap();
    assert_eq!(block.raw_bytes()[0], 0x11);

    let err = stream.finish().unwrap_err();
    assert_eq!(err.error().status_code(), StatusCode::LibUsb);
    assert_eq!(err.stats().buffers_processed, 1);
    let (device, error, stats) = err.into_parts();
    assert_eq!(device.direct().get_sample_type(), SampleType::Raw);
    assert_eq!(error.status_code(), StatusCode::LibUsb);
    assert_eq!(stats.buffers_processed, 1);

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
            ReceiverMode::Off as u16
        ]
    );
}

#[test]
fn owned_raw_rx_stream_finish_success_still_returns_device_and_stats() {
    let control = FakeDevice::with_completions([vec![0x11; DEFAULT_BUFFER_SIZE]]);
    let state = control.state.clone();
    let direct = HydraSdr::from_control(control);
    let mut device = Device::from_direct_without_info(direct);
    device
        .configure(
            &Config::builder()
                .sample_format(SampleFormat::RawAdc)
                .build()
                .unwrap(),
        )
        .unwrap();
    state.borrow_mut().control_requests.clear();

    let stream = device.into_raw_rx_stream_owned().unwrap();
    let (device, stats) = stream.finish().unwrap();

    assert_eq!(device.direct().get_sample_type(), SampleType::Raw);
    assert_eq!(stats, StreamingStats::default());
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
            ReceiverMode::Off as u16
        ]
    );
}

#[test]
fn owned_f32_rx_stream_finish_preserves_device_stats_and_error_on_stop_failure() {
    let control = FakeDevice::with_completions_and_final_receiver_off_error([
        vec![0x80; DEFAULT_BUFFER_SIZE],
    ]);
    let state = control.state.clone();
    let direct = HydraSdr::from_control(control);
    let device = Device::from_direct_without_info(direct);
    let mut stream = device.into_f32_rx_stream_owned().unwrap();

    let mut out = [(0.0, 0.0); 4];
    assert_eq!(stream.read(&mut out, Duration::from_millis(0)).unwrap(), 4);

    let err = stream.finish().unwrap_err();
    assert_eq!(err.error().status_code(), StatusCode::LibUsb);
    assert_eq!(err.stats().buffers_processed, 1);
    let (device, error, stats) = err.into_parts();
    assert_eq!(device.direct().get_sample_type(), SampleType::Float32Iq);
    assert_eq!(error.status_code(), StatusCode::LibUsb);
    assert_eq!(stats.buffers_processed, 1);

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
            ReceiverMode::Off as u16
        ]
    );
}

#[test]
fn owned_f32_rx_stream_finish_success_still_returns_device_and_stats() {
    let control = FakeDevice::with_completions([vec![0x80; DEFAULT_BUFFER_SIZE]]);
    let state = control.state.clone();
    let direct = HydraSdr::from_control(control);
    let device = Device::from_direct_without_info(direct);
    let stream = device.into_f32_rx_stream_owned().unwrap();

    let (device, stats) = stream.finish().unwrap();

    assert_eq!(device.direct().get_sample_type(), SampleType::Float32Iq);
    assert_eq!(stats, StreamingStats::default());
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
            ReceiverMode::Off as u16
        ]
    );
}

#[test]
fn sample_block_reports_raw_view_format_and_drop_count() {
    let raw = [1, 2, 3, 4];
    let block = SampleBlock::new(&raw, SampleFormat::RawAdc, 2, 9);

    assert_eq!(block.raw_bytes(), &raw);
    assert_eq!(block.sample_format(), SampleFormat::RawAdc);
    assert_eq!(block.sample_count(), 2);
    assert_eq!(block.dropped_samples(), 9);
}

fn part_id_serial_response() -> Vec<u8> {
    [
        0x1111_2222u32,
        0x3333_4444,
        0x5555_6666,
        0x7777_8888,
        0x9999_aaaa,
        0xbbbb_cccc,
    ]
    .into_iter()
    .flat_map(u32::to_le_bytes)
    .collect()
}
