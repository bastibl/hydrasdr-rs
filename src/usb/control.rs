//! USB control-transfer encoding and the `nusb` backend implementation.

use std::future::Future;
use std::time::Duration;

use nusb::Endpoint;
#[cfg(not(target_arch = "wasm32"))]
use nusb::MaybeFuture;
use nusb::transfer::{
    Buffer as NusbBuffer, Bulk, ControlIn, ControlOut, ControlType, In, Recipient,
};

use crate::commands::{GainType, ReceiverMode, VendorRequest};
use crate::config::RfPort;
use crate::constants::CTRL_TIMEOUT_MS;
use crate::errors::{Error, Result, StatusCode};
use crate::streaming::{AsyncBulkInBackend, AsyncStreamingBackend, BulkInCompletion};
#[cfg(not(target_arch = "wasm32"))]
use crate::streaming::{BulkInBackend, StreamingBackend};
use crate::types::PartIdSerialNo;

/// Direction of a C-style vendor control transfer.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ControlDirection {
    In,
    Out,
}

/// Encoded vendor control request before conversion into `nusb` transfer structs.
///
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct VendorControlRequest {
    direction: ControlDirection,
    request: VendorRequest,
    value: u16,
    index: u16,
    length: usize,
    data: Vec<u8>,
    pub(crate) timeout: Duration,
}

impl VendorControlRequest {
    /// Build a vendor/device IN request with the default C control timeout.
    pub(crate) fn in_request(
        request: VendorRequest,
        value: u16,
        index: u16,
        length: usize,
    ) -> Self {
        Self::in_request_with_timeout(
            request,
            value,
            index,
            length,
            Duration::from_millis(CTRL_TIMEOUT_MS),
        )
    }

    /// Build a vendor/device IN request with an explicit timeout.
    pub(crate) fn in_request_with_timeout(
        request: VendorRequest,
        value: u16,
        index: u16,
        length: usize,
        timeout: Duration,
    ) -> Self {
        Self {
            direction: ControlDirection::In,
            request,
            value,
            index,
            length,
            data: Vec::new(),
            timeout,
        }
    }

    /// Build a vendor/device OUT request with the default C control timeout.
    pub(crate) fn out_request(
        request: VendorRequest,
        value: u16,
        index: u16,
        data: Vec<u8>,
    ) -> Self {
        Self::out_request_with_timeout(
            request,
            value,
            index,
            data,
            Duration::from_millis(CTRL_TIMEOUT_MS),
        )
    }

    /// Build a vendor/device OUT request with an explicit timeout.
    pub(crate) fn out_request_with_timeout(
        request: VendorRequest,
        value: u16,
        index: u16,
        data: Vec<u8>,
        timeout: Duration,
    ) -> Self {
        let length = data.len();
        Self {
            direction: ControlDirection::Out,
            request,
            value,
            index,
            length,
            data,
            timeout,
        }
    }

    /// Convert this direct request into a `nusb` IN control transfer.
    pub(crate) fn nusb_control_in(&self) -> Result<ControlIn> {
        if self.direction != ControlDirection::In || self.length > u16::MAX as usize {
            return Err(Error::status(StatusCode::InvalidParam));
        }
        Ok(ControlIn {
            control_type: ControlType::Vendor,
            recipient: Recipient::Device,
            request: self.request as u8,
            value: self.value,
            index: self.index,
            length: self.length as u16,
        })
    }

    /// Convert this direct request into a `nusb` OUT control transfer.
    pub(crate) fn nusb_control_out(&self) -> Result<ControlOut<'_>> {
        if self.direction != ControlDirection::Out {
            return Err(Error::status(StatusCode::InvalidParam));
        }
        Ok(ControlOut {
            control_type: ControlType::Vendor,
            recipient: Recipient::Device,
            request: self.request as u8,
            value: self.value,
            index: self.index,
            data: &self.data,
        })
    }

    /// Encode receiver mode selection.
    pub(crate) fn receiver_mode(mode: ReceiverMode) -> Self {
        Self::out_request(VendorRequest::ReceiverMode, mode as u16, 0, Vec::new())
    }

    /// Encode frequency setting as an 8-byte little-endian OUT payload.
    pub(crate) fn set_frequency(freq_hz: u64) -> Self {
        Self::out_request(VendorRequest::SetFreq, 0, 0, freq_hz.to_le_bytes().to_vec())
    }

    /// Encode the C samplerate count query.
    pub(crate) fn get_samplerates_count(extended: bool) -> Self {
        Self::in_request(VendorRequest::GetSamplerates, u16::from(extended), 0, 4)
    }

    /// Encode the C samplerate list query.
    pub(crate) fn get_samplerates(count: u32, extended: bool) -> Self {
        Self::in_request(
            VendorRequest::GetSamplerates,
            u16::from(extended),
            count as u16,
            count as usize * 4,
        )
    }

    /// Encode samplerate selection by index or kHz-derived value.
    pub(crate) fn set_samplerate(index_or_khz: u16, response_len: usize) -> Self {
        Self::in_request(VendorRequest::SetSamplerate, 0, index_or_khz, response_len)
    }

    /// Encode the C bandwidth count query.
    pub(crate) fn get_bandwidths_count() -> Self {
        Self::in_request(VendorRequest::GetBandwidths, 0, 0, 4)
    }

    /// Encode the C bandwidth list query.
    pub(crate) fn get_bandwidths(count: u32) -> Self {
        Self::in_request(
            VendorRequest::GetBandwidths,
            0,
            count as u16,
            count as usize * 4,
        )
    }

    /// Encode bandwidth selection by index or kHz-derived value.
    pub(crate) fn set_bandwidth(index_or_khz: u16) -> Self {
        Self::in_request(VendorRequest::SetBandwidth, 0, index_or_khz, 1)
    }

    /// Encode one of the legacy gain requests.
    pub(crate) fn legacy_gain(request: VendorRequest, value: u8) -> Self {
        Self::in_request(request, 0, value as u16, 1)
    }

    /// Encode the extended gain request.
    pub(crate) fn unified_gain(gain_type: GainType, value: u8) -> Self {
        Self::in_request(VendorRequest::SetGain, gain_type as u16, value as u16, 1)
    }

    /// Encode RF bias tee control.
    pub(crate) fn set_rf_bias(value: u8) -> Self {
        Self::out_request(VendorRequest::SetRfBiasCmd, 0, value as u16, Vec::new())
    }

    /// Encode packed-sample mode control.
    pub(crate) fn set_packing(value: u8) -> Self {
        Self::in_request(VendorRequest::SetPacking, 0, value as u16, 1)
    }

    /// Encode RF input port selection.
    pub(crate) fn set_rf_port(port: RfPort) -> Self {
        Self::in_request(VendorRequest::SetRfPort, 0, port as u16, 1)
    }

    /// Encode board ID read.
    pub(crate) fn board_id_read() -> Self {
        Self::in_request(VendorRequest::BoardIdRead, 0, 0, 1)
    }

    /// Encode firmware version string read.
    pub(crate) fn version_string_read(length: usize) -> Self {
        Self::in_request(VendorRequest::VersionStringRead, 0, 0, length)
    }

    /// Encode board part/serial read.
    pub(crate) fn board_partid_serialno_read() -> Self {
        Self::in_request(VendorRequest::BoardPartIdSerialNoRead, 0, 0, 24)
    }

    /// Encode capability-word read.
    pub(crate) fn get_capabilities(word: u16) -> Self {
        Self::in_request(VendorRequest::GetCapabilities, 0, word, 4)
    }
}

/// Synchronous control-transfer backend for the direct API.
#[cfg(not(target_arch = "wasm32"))]
pub(crate) trait ControlBackend: std::fmt::Debug {
    fn control_in(&self, request: VendorControlRequest) -> Result<Vec<u8>>;
    fn control_out(&self, request: VendorControlRequest) -> Result<()>;
}

/// Async control-transfer backend for the direct API.
pub(crate) trait AsyncControlBackend: std::fmt::Debug {
    fn control_in_async(
        &self,
        request: VendorControlRequest,
    ) -> impl Future<Output = Result<Vec<u8>>> + '_;

    fn control_out_async(
        &self,
        request: VendorControlRequest,
    ) -> impl Future<Output = Result<()>> + '_;
}

/// `nusb` implementation of direct control and streaming backends.
#[derive(Debug)]
pub(crate) struct NusbControl {
    _device: nusb::Device,
    interface: nusb::Interface,
}

/// `nusb` bulk-IN endpoint wrapper used by direct streaming.
#[derive(Debug)]
pub(crate) struct NusbBulkIn {
    endpoint: Endpoint<Bulk, In>,
}

impl NusbControl {
    /// Build a backend from an opened `nusb` device and claimed interface.
    pub(crate) fn new(device: nusb::Device, interface: nusb::Interface) -> Self {
        Self {
            _device: device,
            interface,
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl ControlBackend for NusbControl {
    fn control_in(&self, request: VendorControlRequest) -> Result<Vec<u8>> {
        let control = request.nusb_control_in()?;
        self.interface
            .control_in(control, request.timeout)
            .wait()
            .map_err(Error::from)
    }

    fn control_out(&self, request: VendorControlRequest) -> Result<()> {
        let control = request.nusb_control_out()?;
        self.interface
            .control_out(control, request.timeout)
            .wait()
            .map_err(Error::from)
    }
}

impl AsyncControlBackend for NusbControl {
    async fn control_in_async(&self, request: VendorControlRequest) -> Result<Vec<u8>> {
        let control = request.nusb_control_in()?;
        self.interface
            .control_in(control, request.timeout)
            .await
            .map_err(Error::from)
    }

    async fn control_out_async(&self, request: VendorControlRequest) -> Result<()> {
        let control = request.nusb_control_out()?;
        self.interface
            .control_out(control, request.timeout)
            .await
            .map_err(Error::from)
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl StreamingBackend for NusbControl {
    type BulkIn = NusbBulkIn;

    fn bulk_in(&self, endpoint: u8) -> Result<Self::BulkIn> {
        Ok(NusbBulkIn {
            endpoint: self
                .interface
                .endpoint::<Bulk, In>(endpoint)
                .map_err(Error::from)?,
        })
    }
}

impl AsyncStreamingBackend for NusbControl {
    type BulkIn = NusbBulkIn;

    async fn bulk_in_async(&self, endpoint: u8) -> Result<Self::BulkIn> {
        Ok(NusbBulkIn {
            endpoint: self
                .interface
                .endpoint::<Bulk, In>(endpoint)
                .map_err(Error::from)?,
        })
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl BulkInBackend for NusbBulkIn {
    type Buffer = NusbBuffer;

    fn clear_halt(&mut self) -> Result<()> {
        self.endpoint.clear_halt().wait().map_err(Error::from)
    }

    fn allocate(&self, len: usize) -> Self::Buffer {
        self.endpoint.allocate(len)
    }

    fn submit(&mut self, buffer: Self::Buffer) {
        self.endpoint.submit(buffer);
    }

    fn pending(&self) -> usize {
        self.endpoint.pending()
    }

    fn wait_next_complete(&mut self, timeout: Duration) -> Option<BulkInCompletion<Self::Buffer>> {
        self.endpoint
            .wait_next_complete(timeout)
            .map(|completion| BulkInCompletion {
                buffer: completion.buffer,
                actual_len: completion.actual_len,
                status: completion.status.map_err(Error::from),
            })
    }

    fn cancel_all(&mut self) {
        self.endpoint.cancel_all();
    }
}

impl AsyncBulkInBackend for NusbBulkIn {
    type Buffer = NusbBuffer;

    async fn clear_halt_async(&mut self) -> Result<()> {
        self.endpoint.clear_halt().await.map_err(Error::from)
    }

    fn allocate(&self, len: usize) -> Self::Buffer {
        self.endpoint.allocate(len)
    }

    fn submit(&mut self, buffer: Self::Buffer) {
        self.endpoint.submit(buffer);
    }

    fn pending(&self) -> usize {
        self.endpoint.pending()
    }

    async fn next_complete_async(&mut self) -> BulkInCompletion<Self::Buffer> {
        let completion = self.endpoint.next_complete().await;
        BulkInCompletion {
            buffer: completion.buffer,
            actual_len: completion.actual_len,
            status: completion.status.map_err(Error::from),
        }
    }

    fn cancel_all(&mut self) {
        #[cfg(not(target_arch = "wasm32"))]
        self.endpoint.cancel_all();
    }
}

pub(crate) fn decode_u32_le_words(bytes: &[u8]) -> Result<Vec<u32>> {
    let chunks = bytes.chunks_exact(4);
    if !chunks.remainder().is_empty() {
        return Err(Error::status(StatusCode::LibUsb));
    }
    Ok(chunks
        .map(|chunk| u32::from_le_bytes(chunk.try_into().expect("chunk has four bytes")))
        .collect())
}

pub(crate) fn decode_part_id_serial(bytes: &[u8]) -> Result<PartIdSerialNo> {
    let words = decode_u32_le_words(bytes)?;
    if words.len() < 6 {
        return Err(Error::status(StatusCode::LibUsb));
    }
    Ok(PartIdSerialNo {
        part_id: [words[0], words[1]],
        serial_no: [words[2], words[3], words[4], words[5]],
    })
}
