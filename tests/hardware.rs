use std::sync::{Mutex, MutexGuard};

use hydrasdr_rs::{Device, GainPreset, RfPort, SampleBlock, SampleFormat};

static HARDWARE_TEST_LOCK: Mutex<()> = Mutex::new(());

fn hardware_test_lock() -> MutexGuard<'static, ()> {
    HARDWARE_TEST_LOCK
        .lock()
        .expect("hardware test lock should not be poisoned")
}

#[test]
#[ignore = "requires a connected HydraSDR RFOne and USB permissions; run with `cargo test --test hardware -- --ignored --nocapture`"]
fn hardware_open_and_query_device_info() {
    let _lock = hardware_test_lock();
    let dev = Device::open().expect("open HydraSDR RFOne");
    let info = dev.info();

    assert!(
        info.firmware_version.starts_with("HydraSDR RF"),
        "unexpected version: {}",
        info.firmware_version
    );
    assert!(info.features != 0);
}

#[test]
#[ignore = "requires a connected HydraSDR RFOne and USB permissions; run with `cargo test --test hardware -- --ignored --nocapture`"]
fn hardware_configure_frequency_sample_rate_and_gains() {
    let _lock = hardware_test_lock();
    let mut dev = Device::builder()
        .frequency_hz(100_000_000)
        .sample_rate_hz(10_000_000)
        .sample_format(SampleFormat::RawU8Iq)
        .rf_port(RfPort::Rx0)
        .gain(GainPreset::Linearity(10))
        .bias_tee(false)
        .open()
        .expect("open and configure HydraSDR RFOne");

    let info = dev.refresh_info().expect("refresh device info");
    assert_eq!(info.current_samplerate, 10_000_000);
}

#[test]
#[ignore = "requires a connected HydraSDR RFOne and USB permissions; run with `cargo test --test hardware -- --ignored --nocapture`"]
fn hardware_short_rx_stream_smoke_test() {
    let _lock = hardware_test_lock();
    let mut dev = Device::builder()
        .frequency_hz(100_000_000)
        .sample_rate_hz(10_000_000)
        .sample_format(SampleFormat::RawU8Iq)
        .rf_port(RfPort::Rx0)
        .gain(GainPreset::Sensitivity(8))
        .open()
        .expect("open and configure HydraSDR RFOne");

    let mut callbacks = 0;
    let stats = dev
        .receive_blocks(|transfer: SampleBlock<'_>| {
            callbacks += 1;
            assert!(!transfer.raw_bytes().is_empty());
            assert!(transfer.sample_count() > 0);
            true
        })
        .expect("short RX stream");

    assert_eq!(callbacks, 1);
    assert_eq!(stats.buffers_processed, 1);
}
