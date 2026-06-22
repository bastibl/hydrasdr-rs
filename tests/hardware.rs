use hydrasdr_rs::commands::GainType;
use hydrasdr_rs::device::HydraSdr;
use hydrasdr_rs::streaming::Transfer;
use hydrasdr_rs::types::SampleType;

#[test]
#[ignore = "requires a connected HydraSDR RFOne and USB permissions; run with `cargo test --test hardware -- --ignored --nocapture`"]
fn hardware_open_and_query_board_identity() {
    let mut dev = HydraSdr::open().expect("open HydraSDR RFOne");

    let board_id = dev.board_id_read().expect("read board id");
    let version = dev.version_string_read().expect("read firmware version");
    let part_serial = dev
        .board_partid_serialno_read()
        .expect("read part ID / serial number");
    let info = dev.get_device_info().expect("read device info");

    assert_eq!(board_id, info.board_id);
    assert!(
        version.starts_with("HydraSDR RF"),
        "unexpected version: {version}"
    );
    assert_eq!(part_serial, info.part_serial);
    assert!(info.features != 0);
    dev.close().unwrap();
}

#[test]
#[ignore = "requires a connected HydraSDR RFOne and USB permissions; run with `cargo test --test hardware -- --ignored --nocapture`"]
fn hardware_configure_sample_rate_frequency_and_gains() {
    let mut dev = HydraSdr::open().expect("open HydraSDR RFOne");

    let samplerates = dev.get_samplerates().expect("query samplerates");
    let samplerate = *samplerates
        .first()
        .expect("device reports at least one samplerate");
    dev.set_freq(100_000_000).expect("set frequency");
    dev.set_samplerate(samplerate).expect("set samplerate");
    dev.set_sample_type(SampleType::Float32Iq)
        .expect("set sample type");
    dev.set_lna_gain(8).expect("set LNA gain");
    dev.set_mixer_gain(8).expect("set mixer gain");
    dev.set_vga_gain(8).expect("set VGA gain");
    dev.set_gain(GainType::Linearity, 10)
        .expect("set linearity gain preset");

    dev.close().unwrap();
}

#[test]
#[ignore = "requires a connected HydraSDR RFOne and USB permissions; run with `cargo test --test hardware -- --ignored --nocapture`"]
fn hardware_short_rx_stream_smoke_test() {
    let mut dev = HydraSdr::open().expect("open HydraSDR RFOne");

    let samplerate = *dev
        .get_samplerates()
        .expect("query samplerates")
        .first()
        .expect("device reports at least one samplerate");
    dev.set_freq(100_000_000).expect("set frequency");
    dev.set_samplerate(samplerate).expect("set samplerate");
    dev.set_sample_type(SampleType::Raw)
        .expect("set raw sample type");

    let mut callbacks = 0;
    let stats = dev
        .start_rx(|transfer: &Transfer<'_>| {
            callbacks += 1;
            assert!(!transfer.samples.is_empty());
            assert!(transfer.sample_count > 0);
            1
        })
        .expect("short RX stream");

    assert_eq!(callbacks, 1);
    assert_eq!(stats.buffers_processed, 1);
    assert!(!dev.is_streaming());
    dev.close().unwrap();
}
