//! Smol-based async HydraSDR receive example.
//!
//! Run only when an RFOne is connected and USB permissions are configured.

use futures_lite::future::block_on;
use hydrasdr_rs::{Complex32, Device, GainPreset, RfPort};

const EXAMPLE_FREQ_HZ: u64 = 100_000_000;
const EXAMPLE_SAMPLE_RATE_HZ: u32 = 10_000_000;

fn main() -> hydrasdr_rs::Result<()> {
    block_on(run())
}

async fn run() -> hydrasdr_rs::Result<()> {
    // DeviceBuilder mirrors the synchronous API while staying
    // executor-agnostic at this crate layer.
    let mut dev = Device::builder()
        .frequency_hz(EXAMPLE_FREQ_HZ)
        .sample_rate_hz(EXAMPLE_SAMPLE_RATE_HZ)
        .rf_port(RfPort::Rx0)
        .gain(GainPreset::Linearity(12))
        .bias_tee(false)
        .open()
        .await?;

    println!(
        "opened {} firmware={} serial={:?}",
        dev.info().board_name,
        dev.info().firmware_version,
        dev.info().serial
    );
    println!(
        "configured: freq={EXAMPLE_FREQ_HZ}Hz sample_rate={EXAMPLE_SAMPLE_RATE_HZ}Hz format=F32Iq"
    );

    let mut rx = dev.rx_stream()?;
    rx.start().await?;
    let mut samples = [Complex32::default(); 32];
    let count = rx.read(&mut samples, None).await?;
    let stats = rx.stop().await?;
    drop(rx);
    dev.shutdown().await?;
    println!("rx samples: {count}, first={:?}", samples.first());
    println!("short async RX complete: {stats:?}");

    Ok(())
}
