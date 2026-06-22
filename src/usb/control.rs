use crate::commands::VendorRequest;
use crate::errors::{Error, Result, StatusCode};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ControlDirection {
    In,
    Out,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct VendorControlRequest {
    pub direction: ControlDirection,
    pub request: VendorRequest,
    pub value: u16,
    pub index: u16,
    pub length: usize,
    pub data: Vec<u8>,
}

impl VendorControlRequest {
    pub fn in_request(request: VendorRequest, value: u16, index: u16, length: usize) -> Self {
        Self {
            direction: ControlDirection::In,
            request,
            value,
            index,
            length,
            data: Vec::new(),
        }
    }

    pub fn out_request(request: VendorRequest, value: u16, index: u16, data: Vec<u8>) -> Self {
        let length = data.len();
        Self {
            direction: ControlDirection::Out,
            request,
            value,
            index,
            length,
            data,
        }
    }

    pub fn set_frequency(freq_hz: u64) -> Self {
        Self::out_request(VendorRequest::SetFreq, 0, 0, freq_hz.to_le_bytes().to_vec())
    }

    pub fn get_samplerates_count(extended: bool) -> Self {
        Self::in_request(VendorRequest::GetSamplerates, u16::from(extended), 0, 4)
    }
}

pub fn gpio_port_pin(port: u8, pin: u8) -> Result<u16> {
    if port > 7 || pin > 31 {
        return Err(Error::Status(StatusCode::InvalidParam));
    }
    Ok(((port as u16) << 5) | pin as u16)
}
