use hydrasdr_rs::{
    Config, Device, DeviceBuilder, DeviceDescriptor, DeviceInfo, Error, ErrorKind, F32Iq,
    GainConfig, GainPreset, RawAdc, RfPort, SampleFormat, StageGain,
};

fn assert_f32_builder(_: DeviceBuilder<F32Iq>) {}
fn assert_f32_config(_: Config<F32Iq>) {}
fn assert_raw_builder(_: DeviceBuilder<RawAdc>) {}
fn assert_raw_config(_: Config<RawAdc>) {}

#[allow(dead_code)]
fn assert_mode_specific_device_setters(
    f32_device: &mut Device<F32Iq>,
    raw_device: &mut Device<RawAdc>,
) {
    drop(f32_device.set_bias_tee(true));
    drop(raw_device.set_bias_tee(true));
    drop(raw_device.set_packing(true));
    drop(f32_device.set_decimation_policy(hydrasdr_rs::DecimationPolicy::HighDefinition));
}

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
    assert_eq!(config.rf_port(), RfPort::Rx0);
    assert_eq!(
        config.gain(),
        GainConfig::Stages {
            lna: StageGain::Manual(14),
            mixer: StageGain::Manual(15),
            vga: 6,
        }
    );
    assert!(!config.bias_tee());
}

#[test]
fn public_config_builder_uses_ergonomic_types() {
    let config = Config::builder()
        .frequency_hz(144_500_000)
        .sample_rate_hz(10_000_000)
        .raw_adc()
        .rf_port(RfPort::Rx0)
        .gain(GainPreset::Linearity(12))
        .bias_tee(false)
        .packing(true)
        .build()
        .unwrap();

    assert_eq!(config.frequency_hz(), 144_500_000);
    assert_eq!(config.sample_rate_hz(), 10_000_000);
    assert_eq!(config.rf_port(), RfPort::Rx0);
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
fn public_config_builder_validates_fixed_sample_rate_tables() {
    for rate in [20_000_000, 10_000_000, 5_000_000] {
        assert!(
            Config::builder()
                .raw_adc()
                .sample_rate_hz(rate)
                .build()
                .is_ok()
        );
    }
    for rate in [
        10_000_000, 5_000_000, 2_500_000, 1_250_000, 625_000, 312_500, 156_250, 78_125,
    ] {
        assert!(Config::builder().sample_rate_hz(rate).build().is_ok());
    }

    for rate in [12_000_000, 2_000_000, 39_062, u32::MAX] {
        assert!(
            Config::builder()
                .sample_rate_hz(rate)
                .build()
                .is_err_and(|err| err.kind() == ErrorKind::InvalidConfig)
        );
    }
}

#[test]
fn public_config_builder_validates_manual_gain_ranges() {
    assert!(
        Config::builder()
            .gain(GainConfig::Stages {
                lna: StageGain::Agc,
                mixer: StageGain::Manual(15),
                vga: 15,
            })
            .build()
            .is_ok()
    );

    for gain in [
        GainConfig::Stages {
            lna: StageGain::Manual(15),
            mixer: StageGain::Agc,
            vga: 0,
        },
        GainConfig::Stages {
            lna: StageGain::Agc,
            mixer: StageGain::Manual(16),
            vga: 0,
        },
        GainConfig::Stages {
            lna: StageGain::Agc,
            mixer: StageGain::Agc,
            vga: 16,
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
        rf_ports: Vec::new(),
    };

    assert_eq!(info.serial, Some(0x1234));

    let descriptor = DeviceDescriptor {
        vid: 0x38af,
        pid: 0x0001,
        description: "HydraSDR RFOne Official VID/PID",
        serial: 0,
        product_string: None,
    };

    let serial: u64 = descriptor.serial;
    assert_eq!(serial, 0);
}
