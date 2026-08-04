use hydrasdr_rs::{
    ActiveState, Bandwidth, Config, Device, DeviceBuilder, DeviceInfo, Error, ErrorKind, F32Iq,
    GainConfig, GainPreset, RawAdc, RfPort, SampleFormat,
};

fn assert_f32_builder(_: DeviceBuilder<F32Iq>) {}
fn assert_f32_config(_: Config<F32Iq>) {}
fn assert_raw_builder(_: DeviceBuilder<RawAdc>) {}
fn assert_raw_config(_: Config<RawAdc>) {}

#[test]
fn public_builders_carry_the_selected_sample_mode() {
    assert_f32_builder(Device::builder());
    assert_raw_builder(Device::builder().raw_adc());
    assert_f32_config(Config::builder().build().unwrap());
    assert_raw_config(Config::builder().raw_adc().build().unwrap());
}

#[test]
fn public_default_config_initializes_all_hardware_settings() {
    let config = Config::default();

    assert_eq!(config.sample_format(), SampleFormat::F32Iq);
    assert_eq!(config.rf_port(), Some(RfPort::Rx0));
    assert_eq!(
        config.gain(),
        GainConfig::Manual {
            lna: Some(14),
            mixer: Some(15),
            vga: Some(6),
            lna_agc: Some(false),
            mixer_agc: Some(false),
        }
    );
    assert_eq!(config.bias_tee(), Some(false));
}

#[test]
fn public_config_builder_uses_ergonomic_types() {
    let config = Config::builder()
        .frequency_hz(144_500_000)
        .sample_rate_hz(10_000_000)
        .bandwidth(Bandwidth::Auto)
        .raw_adc()
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
fn public_config_builder_validates_vendor_parameter_ranges() {
    assert!(
        Config::builder()
            .raw_adc()
            .sample_rate_hz(65_535_999)
            .build()
            .is_ok()
    );
    assert!(Config::builder().sample_rate_hz(32_767_999).build().is_ok());
    assert!(Config::builder().bandwidth_hz(65_535_999).build().is_ok());

    assert!(
        Config::builder()
            .raw_adc()
            .sample_rate_hz(65_536_000)
            .build()
            .is_err_and(|err| err.kind() == ErrorKind::InvalidConfig)
    );
    assert!(
        Config::builder()
            .sample_rate_hz(32_768_000)
            .build()
            .is_err_and(|err| err.kind() == ErrorKind::InvalidConfig)
    );
    assert!(
        Config::builder()
            .raw_adc()
            .sample_rate_hz(u32::MAX)
            .build()
            .is_err_and(|err| err.kind() == ErrorKind::InvalidConfig)
    );
    assert!(
        Config::builder()
            .bandwidth_hz(65_536_000)
            .build()
            .is_err_and(|err| err.kind() == ErrorKind::InvalidConfig)
    );
    assert!(
        Config::builder()
            .bandwidth_hz(u32::MAX)
            .build()
            .is_err_and(|err| err.kind() == ErrorKind::InvalidConfig)
    );
}

#[test]
fn public_config_builder_validates_manual_gain_ranges() {
    assert!(
        Config::builder()
            .gain(GainConfig::Manual {
                lna: Some(14),
                mixer: Some(15),
                vga: Some(15),
                lna_agc: Some(true),
                mixer_agc: Some(false),
            })
            .build()
            .is_ok()
    );

    for gain in [
        GainConfig::Manual {
            lna: Some(15),
            mixer: None,
            vga: None,
            lna_agc: None,
            mixer_agc: None,
        },
        GainConfig::Manual {
            lna: None,
            mixer: Some(16),
            vga: None,
            lna_agc: None,
            mixer_agc: None,
        },
        GainConfig::Manual {
            lna: None,
            mixer: None,
            vga: Some(16),
            lna_agc: None,
            mixer_agc: None,
        },
        GainConfig::Preset(GainPreset::Linearity(22)),
    ] {
        assert!(
            Config::builder()
                .gain(gain)
                .build()
                .is_err_and(|err| err.kind() == ErrorKind::InvalidConfig)
        );
    }
}

#[test]
fn public_error_and_metadata_are_ergonomic() {
    let err = Config::builder().sample_rate_hz(9_999).build().unwrap_err();
    assert_eq!(err.kind(), ErrorKind::InvalidConfig);
    assert!(matches!(
        err,
        Error::InvalidConfig {
            field: "sample_rate_hz",
            ..
        }
    ));

    let info = DeviceInfo {
        board_name: "HydraSDR RFOne",
        firmware_version: "test".to_string(),
        serial: Some(0x1234),
        min_frequency: 24_000_000,
        max_frequency: 1_800_000_000,
        rf_ports: Vec::new(),
        active_state: ActiveState::default(),
    };

    assert_eq!(info.serial, Some(0x1234));
    assert_eq!(
        info.active_state.sample_format().unwrap_err().kind(),
        ErrorKind::StateUnavailable
    );
}
