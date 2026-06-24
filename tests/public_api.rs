use hydrasdr_rs::{Bandwidth, Config, DeviceInfo, ErrorKind, GainPreset, RfPort, SampleFormat};

#[test]
fn public_config_builder_uses_ergonomic_types() {
    let config = Config::builder()
        .frequency_hz(144_500_000)
        .sample_rate_hz(10_000_000)
        .bandwidth(Bandwidth::Auto)
        .sample_format(SampleFormat::RawAdc)
        .rf_port(RfPort::Rx0)
        .gain(GainPreset::Linearity(12))
        .bias_tee(false)
        .packing(true)
        .build()
        .unwrap();

    assert_eq!(config.frequency_hz(), 144_500_000);
    assert_eq!(config.sample_rate_hz(), 10_000_000);
    assert_eq!(config.rf_port(), Some(RfPort::Rx0));
    assert_eq!(config.sample_format(), SampleFormat::RawAdc);
    assert!(config.packing());
}

#[test]
fn public_config_builder_validates_rfone_frequency_range() {
    assert!(
        Config::builder()
            .frequency_hz(23_999_999)
            .build()
            .is_err_and(|err| err.kind() == ErrorKind::InvalidConfig)
    );
    assert!(
        Config::builder()
            .frequency_hz(1_800_000_001)
            .build()
            .is_err_and(|err| err.kind() == ErrorKind::InvalidConfig)
    );

    assert!(Config::builder().frequency_hz(24_000_000).build().is_ok());
    assert!(
        Config::builder()
            .frequency_hz(1_800_000_000)
            .build()
            .is_ok()
    );
}

#[test]
fn public_error_and_metadata_are_ergonomic() {
    let err = Config::builder().sample_rate_hz(9_999).build().unwrap_err();
    assert_eq!(err.kind(), ErrorKind::InvalidConfig);

    let info = DeviceInfo {
        board_name: "HydraSDR RFOne",
        firmware_version: "test".to_string(),
        serial: Some(0x1234),
        min_frequency: 24_000_000,
        max_frequency: 1_800_000_000,
        rf_ports: Vec::new(),
        current_config: Some(Config::default()),
    };

    assert_eq!(info.serial, Some(0x1234));
    assert_eq!(
        info.current_config.as_ref().map(Config::sample_format),
        Some(SampleFormat::RawAdc)
    );
}
