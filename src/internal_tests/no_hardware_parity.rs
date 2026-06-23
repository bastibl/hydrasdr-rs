use std::cell::RefCell;

use crate::commands::{GainType, ReceiverMode, RfPort, VendorRequest};
use crate::device::HydraSdr;
use crate::errors::StatusCode;
use crate::types::SampleType;
use crate::usb::control::{ControlBackend, ControlDirection, VendorControlRequest, gpio_port_pin};
use nusb::transfer::{ControlType, Recipient};

#[derive(Debug, Default)]
struct FakeControl {
    requests: RefCell<Vec<VendorControlRequest>>,
    in_responses: RefCell<Vec<Vec<u8>>>,
}

impl FakeControl {
    fn with_in_responses(responses: Vec<Vec<u8>>) -> Self {
        Self {
            requests: RefCell::new(Vec::new()),
            in_responses: RefCell::new(responses),
        }
    }
}

impl ControlBackend for FakeControl {
    fn control_in(&self, request: VendorControlRequest) -> crate::Result<Vec<u8>> {
        self.requests.borrow_mut().push(request);
        if self.in_responses.borrow().is_empty() {
            return Ok(vec![1]);
        }
        Ok(self.in_responses.borrow_mut().remove(0))
    }

    fn control_out(&self, request: VendorControlRequest) -> crate::Result<()> {
        self.requests.borrow_mut().push(request);
        Ok(())
    }
}

#[test]
fn control_request_builders_encode_vendor_device_packets_like_c() {
    let receiver_off = VendorControlRequest::receiver_mode(ReceiverMode::Off);
    assert_eq!(receiver_off.direction, ControlDirection::Out);
    assert_eq!(receiver_off.request, VendorRequest::ReceiverMode);
    assert_eq!(receiver_off.value, 0);
    assert_eq!(receiver_off.index, 0);
    assert!(receiver_off.data.is_empty());
    let nusb_out = receiver_off.nusb_control_out().unwrap();
    assert_eq!(nusb_out.control_type, ControlType::Vendor);
    assert_eq!(nusb_out.recipient, Recipient::Device);
    assert_eq!(nusb_out.request, VendorRequest::ReceiverMode as u8);
    assert_eq!(nusb_out.value, ReceiverMode::Off as u16);
    assert_eq!(nusb_out.index, 0);
    assert!(nusb_out.data.is_empty());

    let set_samplerate = VendorControlRequest::set_samplerate(20_000, 1);
    assert_eq!(set_samplerate.direction, ControlDirection::In);
    assert_eq!(set_samplerate.request, VendorRequest::SetSamplerate);
    assert_eq!(set_samplerate.value, 0);
    assert_eq!(set_samplerate.index, 20_000);
    assert_eq!(set_samplerate.length, 1);
    let nusb_in = set_samplerate.nusb_control_in().unwrap();
    assert_eq!(nusb_in.control_type, ControlType::Vendor);
    assert_eq!(nusb_in.recipient, Recipient::Device);
    assert_eq!(nusb_in.request, VendorRequest::SetSamplerate as u8);
    assert_eq!(nusb_in.value, 0);
    assert_eq!(nusb_in.index, 20_000);
    assert_eq!(nusb_in.length, 1);

    assert_eq!(VendorControlRequest::set_packing(1).index, 1);
    assert_eq!(VendorControlRequest::set_rf_port(RfPort::Rx1).index, 1);
    assert_eq!(
        VendorControlRequest::unified_gain(GainType::Vga, 17).value,
        4
    );
    assert_eq!(
        VendorControlRequest::unified_gain(GainType::Vga, 17).index,
        17
    );
    assert_eq!(gpio_port_pin(3, 9).unwrap(), 0x69);
}

#[test]
fn invalid_parameters_are_rejected_without_usb_side_effects() {
    let control = FakeControl::default();
    let mut dev = HydraSdr::from_control(control);

    assert_eq!(
        dev.set_freq(0).unwrap_err().status_code(),
        StatusCode::InvalidParam
    );
    assert_eq!(
        dev.set_freq(10_000_000_001).unwrap_err().status_code(),
        StatusCode::InvalidParam
    );
    assert_eq!(
        dev.set_sample_type(SampleType::End)
            .unwrap_err()
            .status_code(),
        StatusCode::InvalidParam
    );
    assert_eq!(
        dev.gpio_write(8, 0, 1).unwrap_err().status_code(),
        StatusCode::InvalidParam
    );
    assert_eq!(
        dev.gpiodir_read(0, 32).unwrap_err().status_code(),
        StatusCode::InvalidParam
    );
    assert_eq!(
        dev.spiflash_write(0x10_0000, &[0xaa])
            .unwrap_err()
            .status_code(),
        StatusCode::InvalidParam
    );
    assert_eq!(
        dev.set_gain(GainType::Count, 0).unwrap_err().status_code(),
        StatusCode::InvalidParam
    );
    assert_eq!(
        dev.get_temperature().unwrap_err().status_code(),
        StatusCode::Unsupported
    );

    assert!(dev.control().requests.borrow().is_empty());
}

#[test]
fn rf_port_firmware_rejection_maps_to_invalid_param() {
    let control = FakeControl::with_in_responses(vec![vec![0]]);
    let mut dev = HydraSdr::from_control(control);

    let err = dev.set_rf_port(RfPort::Rx2).unwrap_err();

    assert_eq!(err.status_code(), StatusCode::InvalidParam);
    assert_eq!(
        dev.control().requests.borrow().as_slice(),
        &[VendorControlRequest::set_rf_port(RfPort::Rx2)]
    );
}

#[test]
fn short_control_reads_are_libusb_errors() {
    let control = FakeControl::with_in_responses(vec![Vec::new()]);
    let dev = HydraSdr::from_control(control);

    let err = dev.board_id_read().unwrap_err();

    assert_eq!(err.status_code(), StatusCode::LibUsb);
    assert_eq!(
        dev.control().requests.borrow().as_slice(),
        &[VendorControlRequest::board_id_read()]
    );
}
