use std::cell::RefCell;

use crate::commands::{RfPort, VendorRequest};
use crate::device::HydraSdr;
use crate::errors::StatusCode;
use crate::types::{BoardId, DecimationMode, PartIdSerialNo, SampleType};
use crate::usb::control::{
    ControlBackend, ControlDirection, VendorControlRequest, decode_part_id_serial,
};

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
fn direct_helpers_send_c_style_control_requests() {
    let control = FakeControl::default();
    let mut dev = HydraSdr::from_control(control);

    dev.set_freq(915_000_000).unwrap();
    dev.set_lna_gain(99).unwrap();
    dev.set_rf_bias(1).unwrap();
    dev.set_rf_port(RfPort::Rx2).unwrap();
    dev.clockgen_write(7, 0xaa).unwrap();
    dev.rf_frontend_write(0x1234, 0x55aa).unwrap();
    dev.spiflash_write(0x0f_1234, &[1, 2, 3]).unwrap();

    {
        let requests = dev.control().requests.borrow();
        assert_eq!(requests.len(), 7);

        assert_eq!(
            requests[0],
            VendorControlRequest::set_frequency(915_000_000)
        );
        assert_eq!(
            requests[1],
            VendorControlRequest::legacy_gain(VendorRequest::SetLnaGain, 14)
        );
        assert_eq!(requests[2], VendorControlRequest::set_rf_bias(1));
        assert_eq!(requests[3], VendorControlRequest::set_rf_port(RfPort::Rx2));
        assert_eq!(requests[4], VendorControlRequest::clockgen_write(7, 0xaa));
        assert_eq!(
            requests[5],
            VendorControlRequest::rf_frontend_write(0x1234, 0x55aa)
        );
        assert_eq!(
            requests[6],
            VendorControlRequest::spiflash_write(0x0f_1234, &[1, 2, 3]).unwrap()
        );
    }

    let bad_freq = dev.set_freq(0).unwrap_err();
    assert_eq!(bad_freq.status_code(), StatusCode::InvalidParam);
    let bad_flash = dev.spiflash_read(0x10_0000, 4).unwrap_err();
    assert_eq!(bad_flash.status_code(), StatusCode::InvalidParam);
}

#[test]
fn query_helpers_decode_board_version_serial_and_capabilities() {
    let part_serial_bytes = [
        0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55, 0xcc, 0xbb, 0xaa, 0x99, 0x00, 0xff, 0xee,
        0xdd, 0x04, 0x03, 0x02, 0x01, 0xef, 0xbe, 0xad, 0xde,
    ];
    assert_eq!(
        decode_part_id_serial(&part_serial_bytes).unwrap(),
        PartIdSerialNo {
            part_id: [0x1122_3344, 0x5566_7788],
            serial_no: [0x99aa_bbcc, 0xddee_ff00, 0x0102_0304, 0xdead_beef],
        }
    );

    let mut version = b"HydraSDR RFOne v1.0".to_vec();
    version.resize(255, 0);
    let control = FakeControl::with_in_responses(vec![
        vec![BoardId::HydraSdrRfOneOfficial as u8],
        version,
        part_serial_bytes.to_vec(),
        0x0030_7801u32.to_le_bytes().to_vec(),
    ]);
    let dev = HydraSdr::from_control(control);

    assert_eq!(dev.board_id_read().unwrap(), BoardId::HydraSdrRfOneOfficial);
    assert_eq!(dev.version_string_read().unwrap(), "HydraSDR RFOne v1.0");
    assert_eq!(
        dev.board_partid_serialno_read().unwrap().serial_no[3],
        0xdead_beef
    );
    assert_eq!(dev.get_capabilities().unwrap(), 0x0030_7801);

    let requests = dev.control().requests.borrow();
    assert_eq!(requests[0], VendorControlRequest::board_id_read());
    assert_eq!(requests[1].request, VendorRequest::VersionStringRead);
    assert_eq!(requests[1].direction, ControlDirection::In);
    assert_eq!(
        requests[2],
        VendorControlRequest::board_partid_serialno_read()
    );
    assert_eq!(requests[3], VendorControlRequest::get_capabilities(0));
}

#[test]
fn sample_rate_and_bandwidth_helpers_use_count_then_list_protocol() {
    let control = FakeControl::with_in_responses(vec![
        2u32.to_le_bytes().to_vec(),
        [10_000_000u32.to_le_bytes(), 20_000_000u32.to_le_bytes()].concat(),
        vec![1],
        vec![1],
        2u32.to_le_bytes().to_vec(),
        [1_750_000u32.to_le_bytes(), 2_500_000u32.to_le_bytes()].concat(),
        vec![1],
    ]);
    let mut dev = HydraSdr::from_control(control);

    assert_eq!(
        dev.get_samplerates().unwrap(),
        vec![
            20_000_000, 10_000_000, 5_000_000, 2_500_000, 1_250_000, 625_000, 312_500, 156_250,
        ]
    );
    dev.set_samplerate(20_000_000).unwrap();
    dev.set_samplerate(2_500_000).unwrap();
    assert_eq!(dev.get_bandwidths().unwrap(), vec![1_750_000, 2_500_000]);
    dev.set_bandwidth(2_500_000).unwrap();

    let requests = dev.control().requests.borrow();
    assert_eq!(
        requests[0],
        VendorControlRequest::get_samplerates_count(false)
    );
    assert_eq!(requests[1], VendorControlRequest::get_samplerates(2, false));
    assert_eq!(requests[2], VendorControlRequest::set_samplerate(1, 1));
    assert_eq!(requests[3], VendorControlRequest::set_samplerate(0, 1));
    assert_eq!(requests[4], VendorControlRequest::get_bandwidths_count());
    assert_eq!(requests[5], VendorControlRequest::get_bandwidths(2));
    assert_eq!(requests[6], VendorControlRequest::set_bandwidth(1));
}

#[test]
fn raw_sample_type_exposes_hardware_sample_rates_only() {
    let control = FakeControl::with_in_responses(vec![
        2u32.to_le_bytes().to_vec(),
        [10_000_000u32.to_le_bytes(), 20_000_000u32.to_le_bytes()].concat(),
    ]);
    let mut dev = HydraSdr::from_control(control);
    dev.set_sample_type(SampleType::Raw).unwrap();

    assert_eq!(dev.get_samplerates().unwrap(), vec![10_000_000, 20_000_000]);
}

#[test]
fn decimation_mode_controls_virtual_iq_rate_selection() {
    let control = FakeControl::with_in_responses(vec![
        3u32.to_le_bytes().to_vec(),
        [
            10_000_000u32.to_le_bytes(),
            5_000_000u32.to_le_bytes(),
            2_500_000u32.to_le_bytes(),
        ]
        .concat(),
        vec![1],
        vec![1],
    ]);
    let mut dev = HydraSdr::from_control(control);

    dev.get_samplerates().unwrap();
    dev.set_samplerate(2_500_000).unwrap();
    assert_eq!(dev.decimation_mode(), DecimationMode::LowBandwidth);

    dev.set_decimation_mode(DecimationMode::HighDefinition)
        .unwrap();
    assert_eq!(dev.decimation_mode(), DecimationMode::HighDefinition);

    let requests = dev.control().requests.borrow();
    assert_eq!(requests[2], VendorControlRequest::set_samplerate(2, 1));
    assert_eq!(requests[3], VendorControlRequest::set_samplerate(0, 1));
}

#[test]
fn hardware_open_path_is_available_but_ignored_by_default() {
    if std::env::var_os("HYDRASDR_RUN_HARDWARE_TESTS").is_none() {
        return;
    }

    let dev = HydraSdr::open().expect("HydraSDR hardware open should work when explicitly enabled");
    let version = dev.version_string_read().unwrap();
    assert!(version.starts_with("HydraSDR RF"));
    dev.close().unwrap();
}
