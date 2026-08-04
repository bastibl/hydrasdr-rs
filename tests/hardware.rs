use std::sync::{Mutex, MutexGuard};
use std::time::Duration;

use hydrasdr_rs::{Device, GainConfig, GainPreset, MaybeFuture, RfPort};

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
    let dev = Device::open().wait().expect("open HydraSDR RFOne");
    let info = dev.info();

    assert!(
        info.firmware_version.starts_with("HydraSDR RF"),
        "unexpected version: {}",
        info.firmware_version
    );
    assert_eq!(
        info.rf_ports
            .iter()
            .map(|port| (port.port, port.name))
            .collect::<Vec<_>>(),
        [(RfPort::Rx0, "ANT")]
    );
    let config = dev.config();
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
    dev.shutdown().wait().expect("explicitly shut down device");
}

#[test]
#[ignore = "requires a connected HydraSDR RFOne and USB permissions; run with `cargo test --test hardware -- --ignored --nocapture`"]
fn hardware_configure_frequency_sample_rate_and_gains() {
    let _lock = hardware_test_lock();
    let dev = Device::builder()
        .frequency_hz(100_000_000)
        .sample_rate_hz(10_000_000)
        .raw_adc()
        .rf_port(RfPort::Rx0)
        .gain(GainPreset::Linearity(10))
        .bias_tee(false)
        .open()
        .wait()
        .expect("open and configure HydraSDR RFOne");

    assert_eq!(dev.config().sample_rate_hz(), 10_000_000);
}

#[test]
#[ignore = "requires a connected HydraSDR RFOne and USB permissions; run with `cargo test --test hardware -- --ignored --nocapture`"]
fn hardware_short_rx_stream_smoke_test() {
    let _lock = hardware_test_lock();
    let dev = Device::builder()
        .frequency_hz(100_000_000)
        .sample_rate_hz(10_000_000)
        .raw_adc()
        .rf_port(RfPort::Rx0)
        .gain(GainPreset::Sensitivity(8))
        .open()
        .wait()
        .expect("open and configure HydraSDR RFOne");

    let mut rx = dev.rx_stream().expect("claim RX stream");
    rx.start().wait().expect("start RX stream");
    {
        let block = rx
            .next_block(Duration::from_secs(1))
            .wait()
            .expect("read RX block")
            .expect("one RX block");
        assert!(!block.raw_bytes().is_empty());
        assert!(block.sample_count() > 0);
    }
    let stats = rx.stop().wait().expect("finish RX stream");

    assert_eq!(stats.buffers_processed, 1);
}

#[test]
#[ignore = "requires a connected HydraSDR RFOne and USB permissions; run with `cargo test --test hardware -- --ignored --nocapture`"]
fn hardware_f32_rx_stream_smoke_test() {
    let _lock = hardware_test_lock();
    let dev = Device::builder()
        .frequency_hz(100_000_000)
        .sample_rate_hz(10_000_000)
        .rf_port(RfPort::Rx0)
        .gain(GainPreset::Sensitivity(8))
        .open()
        .wait()
        .expect("open and configure HydraSDR RFOne");

    let mut rx = dev.rx_stream().expect("claim F32 IQ stream");
    rx.start().wait().expect("start F32 IQ stream");
    let mut samples = [hydrasdr_rs::Complex32::default(); 32];
    let count = rx
        .read(&mut samples, std::time::Duration::from_secs(1))
        .wait()
        .expect("read F32 IQ samples");
    let stats = rx.stop().wait().expect("stop F32 IQ stream");

    assert!(count > 0);
    assert!(
        samples[..count]
            .iter()
            .all(|sample| sample.re.is_finite() && sample.im.is_finite())
    );
    assert!(stats.buffers_processed > 0);
}
