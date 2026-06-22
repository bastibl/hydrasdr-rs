use hydrasdr_rs::commands::{Capability, GainType, ReceiverMode, RfPort, VendorRequest};
use hydrasdr_rs::constants::{
    HYDRASDR_BANDWIDTH_AUTO, HYDRASDR_VER_MAJOR, HYDRASDR_VER_MINOR, HYDRASDR_VER_REVISION,
    HYDRASDR_VERSION, HYDRASDR_VERSION_NUM, make_version,
};
use hydrasdr_rs::discovery::{USB_DEVICE_IDS, parse_hydrasdr_serial};
use hydrasdr_rs::errors::{StatusCode, error_name};
use hydrasdr_rs::rfone::{RFONE_HARDCODED_CAPS, RFONE_SAMPLE_TYPES, RFONE_SPEC};
use hydrasdr_rs::types::{BoardId, DecimationMode, SampleType, board_id_name_raw, sample_type_bit};
use hydrasdr_rs::usb::control::{ControlDirection, VendorControlRequest, gpio_port_pin};

#[test]
fn version_constants_match_c_header() {
    assert_eq!(HYDRASDR_VERSION, "1.1.2");
    assert_eq!(HYDRASDR_VER_MAJOR, 1);
    assert_eq!(HYDRASDR_VER_MINOR, 1);
    assert_eq!(HYDRASDR_VER_REVISION, 2);
    assert_eq!(make_version(1, 1, 2), 0x0101_0002);
    assert_eq!(HYDRASDR_VERSION_NUM, 0x0101_0002);
    assert_eq!(HYDRASDR_BANDWIDTH_AUTO, u32::MAX);
}

#[test]
fn status_codes_and_error_names_match_c_api() {
    assert_eq!(StatusCode::Success.code(), 0);
    assert_eq!(StatusCode::True.code(), 1);
    assert_eq!(StatusCode::InvalidParam.code(), -2);
    assert_eq!(StatusCode::NotFound.code(), -5);
    assert_eq!(StatusCode::Busy.code(), -6);
    assert_eq!(StatusCode::NoMem.code(), -11);
    assert_eq!(StatusCode::Unsupported.code(), -12);
    assert_eq!(StatusCode::LibUsb.code(), -1000);
    assert_eq!(StatusCode::Thread.code(), -1001);
    assert_eq!(StatusCode::StreamingThreadErr.code(), -1002);
    assert_eq!(StatusCode::StreamingStopped.code(), -1003);
    assert_eq!(StatusCode::Other.code(), -9999);

    assert_eq!(StatusCode::try_from(-6), Ok(StatusCode::Busy));
    assert!(StatusCode::try_from(-7).is_err());
    assert_eq!(error_name(StatusCode::Busy), "HYDRASDR_ERROR_BUSY");
    assert_eq!(StatusCode::name_for_code(-7), "hydrasdr unknown error");
}

#[test]
fn board_and_sample_type_values_match_c_enums() {
    assert_eq!(BoardId::ProtoHydraSdr as u8, 0);
    assert_eq!(BoardId::HydraSdrRfOneOfficial as u8, 1);
    assert_eq!(BoardId::Invalid as u8, 0xff);
    assert_eq!(board_id_name_raw(0), "HydraSDR RFOne Legacy VID/PID");
    assert_eq!(board_id_name_raw(1), "HydraSDR RFOne Official VID/PID");
    assert_eq!(board_id_name_raw(0xff), "Invalid Board ID");
    assert_eq!(board_id_name_raw(2), "Unknown Board ID");

    assert_eq!(SampleType::Float32Iq as u8, 0);
    assert_eq!(SampleType::Float32Real as u8, 1);
    assert_eq!(SampleType::Int16Iq as u8, 2);
    assert_eq!(SampleType::Int16Real as u8, 3);
    assert_eq!(SampleType::Uint16Real as u8, 4);
    assert_eq!(SampleType::Raw as u8, 5);
    assert_eq!(SampleType::Int8Iq as u8, 6);
    assert_eq!(SampleType::Uint8Iq as u8, 7);
    assert_eq!(SampleType::Int8Real as u8, 8);
    assert_eq!(SampleType::Uint8Real as u8, 9);
    assert_eq!(SampleType::End as u8, 10);
    assert_eq!(sample_type_bit(SampleType::Raw), 1 << 5);

    assert_eq!(DecimationMode::LowBandwidth as u8, 0);
    assert_eq!(DecimationMode::HighDefinition as u8, 1);
}

#[test]
fn vendor_request_and_capability_values_match_c_commands() {
    assert_eq!(ReceiverMode::Off as u8, 0);
    assert_eq!(ReceiverMode::Rx as u8, 1);
    assert_eq!(VendorRequest::Reset as u8, 0);
    assert_eq!(VendorRequest::ReceiverMode as u8, 1);
    assert_eq!(VendorRequest::VersionStringRead as u8, 10);
    assert_eq!(VendorRequest::SetSamplerate as u8, 12);
    assert_eq!(VendorRequest::SetFreq as u8, 13);
    assert_eq!(VendorRequest::GetSamplerates as u8, 25);
    assert_eq!(VendorRequest::SetRfPort as u8, 28);
    assert_eq!(VendorRequest::GetCapabilities as u8, 29);
    assert_eq!(VendorRequest::SetGain as u8, 33);
    assert_eq!(VendorRequest::VendorRequestCount as u8, 34);

    assert_eq!(Capability::LnaGain.bits(), 1 << 0);
    assert_eq!(Capability::BiasTee.bits(), 1 << 11);
    assert_eq!(Capability::Rx.bits(), 1 << 20);
    assert_eq!(Capability::ExtendedGain.bits(), 1 << 22);

    assert_eq!(GainType::Lna as u8, 0);
    assert_eq!(GainType::Sensitivity as u8, 6);
    assert_eq!(GainType::FilterAgc as u8, 10);
    assert_eq!(GainType::Count as u8, 11);

    assert_eq!(RfPort::Rx0 as u8, 0);
    assert_eq!(RfPort::Rx2 as u8, 2);
    assert_eq!(RfPort::Max as u8, 31);
}

#[test]
fn rfone_static_spec_matches_c_driver_constants() {
    assert_eq!(USB_DEVICE_IDS.len(), 2);
    assert_eq!(USB_DEVICE_IDS[0].vid, 0x1d50);
    assert_eq!(USB_DEVICE_IDS[0].pid, 0x60a1);
    assert_eq!(USB_DEVICE_IDS[1].vid, 0x38af);
    assert_eq!(USB_DEVICE_IDS[1].pid, 0x0001);

    assert_eq!(RFONE_SPEC.transfer_count, 16);
    assert_eq!(RFONE_SPEC.default_buffer_size, 262_144);
    assert_eq!(RFONE_SPEC.packed_buffer_size, 147_456);
    assert_eq!(RFONE_SPEC.bulk_rx_endpoint, 0x81);
    assert_eq!(RFONE_SPEC.min_frequency_hz, 24_000_000);
    assert_eq!(RFONE_SPEC.max_frequency_hz, 1_800_000_000);
    assert_eq!(RFONE_SPEC.rf_port_count, 3);
    assert_eq!(RFONE_SPEC.gpio_count, 18);

    let expected_sample_types = sample_type_bit(SampleType::Float32Iq)
        | sample_type_bit(SampleType::Float32Real)
        | sample_type_bit(SampleType::Int16Iq)
        | sample_type_bit(SampleType::Int16Real)
        | sample_type_bit(SampleType::Uint16Real)
        | sample_type_bit(SampleType::Raw);
    assert_eq!(RFONE_SAMPLE_TYPES, expected_sample_types);
    assert_eq!(
        RFONE_HARDCODED_CAPS & Capability::Rx.bits(),
        Capability::Rx.bits()
    );
    assert_eq!(RFONE_HARDCODED_CAPS & Capability::RfGain.bits(), 0);
}

#[test]
fn no_hardware_helpers_pack_usb_fields_like_c_driver() {
    assert_eq!(
        parse_hydrasdr_serial("HYDRASDR SN: 0123456789ABCDEF"),
        Some(0x0123_4567_89ab_cdef)
    );
    assert_eq!(parse_hydrasdr_serial("not a hydrasdr serial"), None);

    let freq = VendorControlRequest::set_frequency(1_234_567_890);
    assert_eq!(freq.direction, ControlDirection::Out);
    assert_eq!(freq.request, VendorRequest::SetFreq);
    assert_eq!(freq.value, 0);
    assert_eq!(freq.index, 0);
    assert_eq!(freq.data, 1_234_567_890u64.to_le_bytes());

    let samplerates_count = VendorControlRequest::get_samplerates_count(false);
    assert_eq!(samplerates_count.direction, ControlDirection::In);
    assert_eq!(samplerates_count.request, VendorRequest::GetSamplerates);
    assert_eq!(samplerates_count.value, 0);
    assert_eq!(samplerates_count.index, 0);
    assert_eq!(samplerates_count.length, 4);

    assert_eq!(gpio_port_pin(7, 31).unwrap(), 0xff);
    assert!(gpio_port_pin(8, 0).is_err());
    assert!(gpio_port_pin(0, 32).is_err());
}
