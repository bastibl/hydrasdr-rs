use hydrasdr_rs::{
    Bandwidth, Config, Device, DeviceSelector, GainPreset, RfPort, SampleBlock, SampleFormat,
};

#[test]
fn public_config_builder_uses_ergonomic_types() {
    assert_eq!(Device::builder().selector(), DeviceSelector::First);
    assert_eq!(
        Device::builder().serial(0x0123_4567_89ab_cdef).selector(),
        DeviceSelector::Serial(0x0123_4567_89ab_cdef)
    );

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
fn public_sample_block_is_raw_view_with_metadata() {
    let raw = [1, 2, 3, 4];
    let block = SampleBlock::new(&raw, SampleFormat::RawAdc, 2, 9);

    assert_eq!(block.raw_bytes(), &raw);
    assert_eq!(block.sample_format(), SampleFormat::RawAdc);
    assert_eq!(block.sample_count(), 2);
    assert_eq!(block.dropped_samples(), 9);
}
