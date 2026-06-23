//! USB control-transfer encoding and the `nusb` backend implementation.

use std::future::Future;
use std::time::Duration;

use nusb::Endpoint;
use nusb::MaybeFuture;
use nusb::transfer::{
    Buffer as NusbBuffer, Bulk, ControlIn, ControlOut, ControlType, In, Recipient,
};

use crate::commands::{GainType, ReceiverMode, RfPort, VendorRequest};
use crate::constants::{CTRL_TIMEOUT_CHIP_ERASE_MS, CTRL_TIMEOUT_MS};
use crate::errors::{Error, Result, StatusCode};
use crate::streaming::{
    AsyncBulkInBackend, AsyncStreamingBackend, BulkInBackend, BulkInCompletion, StreamingBackend,
};
use crate::types::PartIdSerialNo;

/// Direction of a C-style vendor control transfer.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ControlDirection {
    In,
    Out,
}

/// Encoded vendor control request before conversion into `nusb` transfer structs.
///
/// Keeping this public helps parity tests compare the Rust request packing with the C driver.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct VendorControlRequest {
    pub direction: ControlDirection,
    pub request: VendorRequest,
    pub value: u16,
    pub index: u16,
    pub length: usize,
    pub data: Vec<u8>,
    pub timeout: Duration,
}

impl VendorControlRequest {
    /// Build a vendor/device IN request with the default C control timeout.
    pub fn in_request(request: VendorRequest, value: u16, index: u16, length: usize) -> Self {
        Self::in_request_with_timeout(
            request,
            value,
            index,
            length,
            Duration::from_millis(CTRL_TIMEOUT_MS),
        )
    }

    /// Build a vendor/device IN request with an explicit timeout.
    pub fn in_request_with_timeout(
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
    pub fn out_request(request: VendorRequest, value: u16, index: u16, data: Vec<u8>) -> Self {
        Self::out_request_with_timeout(
            request,
            value,
            index,
            data,
            Duration::from_millis(CTRL_TIMEOUT_MS),
        )
    }

    /// Build a vendor/device OUT request with an explicit timeout.
    pub fn out_request_with_timeout(
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
    pub fn nusb_control_in(&self) -> Result<ControlIn> {
        if self.direction != ControlDirection::In || self.length > u16::MAX as usize {
            return Err(Error::Status(StatusCode::InvalidParam));
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
    pub fn nusb_control_out(&self) -> Result<ControlOut<'_>> {
        if self.direction != ControlDirection::Out {
            return Err(Error::Status(StatusCode::InvalidParam));
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
    pub fn receiver_mode(mode: ReceiverMode) -> Self {
        Self::out_request(VendorRequest::ReceiverMode, mode as u16, 0, Vec::new())
    }

    /// Encode frequency setting as an 8-byte little-endian OUT payload.
    pub fn set_frequency(freq_hz: u64) -> Self {
        Self::out_request(VendorRequest::SetFreq, 0, 0, freq_hz.to_le_bytes().to_vec())
    }

    /// Encode the C samplerate count query.
    pub fn get_samplerates_count(extended: bool) -> Self {
        Self::in_request(VendorRequest::GetSamplerates, u16::from(extended), 0, 4)
    }

    /// Encode the C samplerate list query.
    pub fn get_samplerates(count: u32, extended: bool) -> Self {
        Self::in_request(
            VendorRequest::GetSamplerates,
            u16::from(extended),
            count as u16,
            count as usize * 4,
        )
    }

    /// Encode samplerate selection by index or kHz-derived value.
    pub fn set_samplerate(index_or_khz: u32, response_len: usize) -> Self {
        Self::in_request(
            VendorRequest::SetSamplerate,
            0,
            index_or_khz as u16,
            response_len,
        )
    }

    /// Encode the C bandwidth count query.
    pub fn get_bandwidths_count() -> Self {
        Self::in_request(VendorRequest::GetBandwidths, 0, 0, 4)
    }

    /// Encode the C bandwidth list query.
    pub fn get_bandwidths(count: u32) -> Self {
        Self::in_request(
            VendorRequest::GetBandwidths,
            0,
            count as u16,
            count as usize * 4,
        )
    }

    /// Encode bandwidth selection by index or kHz-derived value.
    pub fn set_bandwidth(index_or_khz: u32) -> Self {
        Self::in_request(VendorRequest::SetBandwidth, 0, index_or_khz as u16, 1)
    }

    /// Encode one of the legacy gain requests.
    pub fn legacy_gain(request: VendorRequest, value: u8) -> Self {
        Self::in_request(request, 0, value as u16, 1)
    }

    /// Encode the extended gain request.
    pub fn unified_gain(gain_type: GainType, value: u8) -> Self {
        Self::in_request(VendorRequest::SetGain, gain_type as u16, value as u16, 1)
    }

    /// Encode RF bias tee control.
    pub fn set_rf_bias(value: u8) -> Self {
        Self::out_request(VendorRequest::SetRfBiasCmd, 0, value as u16, Vec::new())
    }

    /// Encode packed-sample mode control.
    pub fn set_packing(value: u8) -> Self {
        Self::in_request(VendorRequest::SetPacking, 0, value as u16, 1)
    }

    /// Encode RF input port selection.
    pub fn set_rf_port(port: RfPort) -> Self {
        Self::in_request(VendorRequest::SetRfPort, 0, port as u16, 1)
    }

    /// Encode device reset.
    pub fn reset() -> Self {
        Self::in_request(VendorRequest::Reset, 0, 0, 1)
    }

    /// Encode board ID read.
    pub fn board_id_read() -> Self {
        Self::in_request(VendorRequest::BoardIdRead, 0, 0, 1)
    }

    /// Encode firmware version string read.
    pub fn version_string_read(length: usize) -> Self {
        Self::in_request(VendorRequest::VersionStringRead, 0, 0, length)
    }

    /// Encode board part/serial read.
    pub fn board_partid_serialno_read() -> Self {
        Self::in_request(VendorRequest::BoardPartIdSerialNoRead, 0, 0, 24)
    }

    /// Encode capability-word read.
    pub fn get_capabilities(word: u16) -> Self {
        Self::in_request(VendorRequest::GetCapabilities, 0, word, 4)
    }

    /// Encode GPIO write using C port/pin packing.
    pub fn gpio_write(port: u8, pin: u8, value: u8) -> Result<Self> {
        Ok(Self::out_request(
            VendorRequest::GpioWrite,
            value as u16,
            gpio_port_pin(port, pin)?,
            Vec::new(),
        ))
    }

    /// Encode GPIO read using C port/pin packing.
    pub fn gpio_read(port: u8, pin: u8) -> Result<Self> {
        Ok(Self::in_request(
            VendorRequest::GpioRead,
            0,
            gpio_port_pin(port, pin)?,
            1,
        ))
    }

    /// Encode GPIO direction write using C port/pin packing.
    pub fn gpiodir_write(port: u8, pin: u8, value: u8) -> Result<Self> {
        Ok(Self::out_request(
            VendorRequest::GpioDirWrite,
            value as u16,
            gpio_port_pin(port, pin)?,
            Vec::new(),
        ))
    }

    /// Encode GPIO direction read using C port/pin packing.
    pub fn gpiodir_read(port: u8, pin: u8) -> Result<Self> {
        Ok(Self::in_request(
            VendorRequest::GpioDirRead,
            0,
            gpio_port_pin(port, pin)?,
            1,
        ))
    }

    /// Encode clock-generator register write.
    pub fn clockgen_write(reg: u8, value: u8) -> Self {
        Self::out_request(
            VendorRequest::ClockgenWrite,
            value as u16,
            reg as u16,
            Vec::new(),
        )
    }

    /// Encode clock-generator register read.
    pub fn clockgen_read(reg: u8) -> Self {
        Self::in_request(VendorRequest::ClockgenRead, 0, reg as u16, 1)
    }

    /// Encode RF frontend register write.
    pub fn rf_frontend_write(reg: u16, value: u32) -> Self {
        Self::out_request(
            VendorRequest::RfFrontendWrite,
            (value & 0xff) as u16,
            reg & 0xff,
            Vec::new(),
        )
    }

    /// Encode RF frontend register read.
    pub fn rf_frontend_read(reg: u16) -> Self {
        Self::in_request(VendorRequest::RfFrontendRead, 0, reg & 0xff, 1)
    }

    /// Encode whole-chip SPI flash erase with the C long timeout.
    pub fn spiflash_erase() -> Self {
        Self::out_request_with_timeout(
            VendorRequest::SpiFlashErase,
            0,
            0,
            Vec::new(),
            Duration::from_millis(CTRL_TIMEOUT_CHIP_ERASE_MS),
        )
    }

    /// Encode SPI flash sector erase with the C long timeout.
    pub fn spiflash_erase_sector(sector: u16) -> Self {
        Self::out_request_with_timeout(
            VendorRequest::SpiFlashEraseSector,
            sector,
            0,
            Vec::new(),
            Duration::from_millis(CTRL_TIMEOUT_CHIP_ERASE_MS),
        )
    }

    /// Encode SPI flash write after validating the C-supported address range.
    pub fn spiflash_write(addr: u32, data: &[u8]) -> Result<Self> {
        validate_spiflash_addr(addr)?;
        Ok(Self::out_request_with_timeout(
            VendorRequest::SpiFlashWrite,
            (addr >> 16) as u16,
            (addr & 0xffff) as u16,
            data.to_vec(),
            Duration::from_millis(0),
        ))
    }

    /// Encode SPI flash read after validating the C-supported address range.
    pub fn spiflash_read(addr: u32, len: u16) -> Result<Self> {
        validate_spiflash_addr(addr)?;
        Ok(Self::in_request_with_timeout(
            VendorRequest::SpiFlashRead,
            (addr >> 16) as u16,
            (addr & 0xffff) as u16,
            len as usize,
            Duration::from_millis(0),
        ))
    }
}

/// Synchronous control-transfer backend for the direct API.
pub trait ControlBackend: std::fmt::Debug {
    fn control_in(&self, request: VendorControlRequest) -> Result<Vec<u8>>;
    fn control_out(&self, request: VendorControlRequest) -> Result<()>;
}

/// Async control-transfer backend for the direct API.
pub trait AsyncControlBackend: std::fmt::Debug {
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
pub struct NusbControl {
    _device: nusb::Device,
    interface: nusb::Interface,
}

/// `nusb` bulk-IN endpoint wrapper used by direct streaming.
#[derive(Debug)]
pub struct NusbBulkIn {
    endpoint: Endpoint<Bulk, In>,
}

impl NusbControl {
    /// Build a backend from an opened `nusb` device and claimed interface.
    pub fn new(device: nusb::Device, interface: nusb::Interface) -> Self {
        Self {
            _device: device,
            interface,
        }
    }
}

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
        self.endpoint.cancel_all();
    }
}

pub fn gpio_port_pin(port: u8, pin: u8) -> Result<u16> {
    if port > 7 || pin > 31 {
        return Err(Error::Status(StatusCode::InvalidParam));
    }
    Ok(((port as u16) << 5) | pin as u16)
}

pub fn validate_spiflash_addr(addr: u32) -> Result<()> {
    if addr > 0x0f_ffff {
        return Err(Error::Status(StatusCode::InvalidParam));
    }
    Ok(())
}

pub fn decode_u32_le_words(bytes: &[u8]) -> Result<Vec<u32>> {
    let chunks = bytes.chunks_exact(4);
    if !chunks.remainder().is_empty() {
        return Err(Error::Status(StatusCode::LibUsb));
    }
    Ok(chunks
        .map(|chunk| u32::from_le_bytes(chunk.try_into().expect("chunk has four bytes")))
        .collect())
}

pub fn decode_part_id_serial(bytes: &[u8]) -> Result<PartIdSerialNo> {
    let words = decode_u32_le_words(bytes)?;
    if words.len() < 6 {
        return Err(Error::Status(StatusCode::LibUsb));
    }
    Ok(PartIdSerialNo {
        part_id: [words[0], words[1]],
        serial_no: [words[2], words[3], words[4], words[5]],
    })
}
