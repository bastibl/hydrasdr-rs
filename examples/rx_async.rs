//! Async HydraSDR receive example.
//!
//! The default invocation prints usage and exits without touching USB. Pass
//! `--run` only when an RFOne is connected and USB permissions are configured.

use std::env;

#[cfg(any(feature = "smol", feature = "tokio"))]
use futures_lite::future::block_on;
#[cfg(any(feature = "smol", feature = "tokio"))]
use hydrasdr_rs::{Device, GainPreset, RfPort, SampleFormat};

#[cfg(any(feature = "smol", feature = "tokio"))]
const EXAMPLE_FREQ_HZ: u64 = 100_000_000;
#[cfg(any(feature = "smol", feature = "tokio"))]
const EXAMPLE_SAMPLE_RATE_HZ: u32 = 10_000_000;

fn main() -> hydrasdr_rs::Result<()> {
    let args: Vec<String> = env::args().skip(1).collect();
    if !args.iter().any(|arg| arg == "--run") {
        print_usage();
        return Ok(());
    }

    #[cfg(not(any(feature = "smol", feature = "tokio")))]
    {
        eprintln!(
            "The async example needs nusb runtime integration.\n\
             Run with one feature enabled, for example:\n\
               cargo run --features smol --example rx_async -- --run --rx"
        );
        Ok(())
    }

    #[cfg(any(feature = "smol", feature = "tokio"))]
    block_on(run(args))
}

#[cfg(any(feature = "smol", feature = "tokio"))]
async fn run(args: Vec<String>) -> hydrasdr_rs::Result<()> {
    let run_rx = args.iter().any(|arg| arg == "--rx");
    // DeviceBuilder mirrors the synchronous API while staying
    // executor-agnostic at this crate layer.
    let dev = Device::builder()
        .frequency_hz(EXAMPLE_FREQ_HZ)
        .sample_rate_hz(EXAMPLE_SAMPLE_RATE_HZ)
        .sample_format(SampleFormat::F32Iq)
        .rf_port(RfPort::Rx0)
        .gain(GainPreset::Linearity(12))
        .bias_tee(false)
        .open_async()
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

    if run_rx {
        let mut rx = dev.into_async_f32_rx_stream();
        rx.start().await?;
        let mut samples = [(0.0, 0.0); 32];
        let count = rx.read(&mut samples).await?;
        let stats = rx.finish().await?;
        let _dev = rx.into_device();
        println!("rx samples: {count}, first={:?}", samples.first());
        println!("short async RX complete: {stats:?}");
    } else {
        println!("RX not started; pass --rx with --run for a one-buffer async smoke stream.");
    }

    Ok(())
}

fn print_usage() {
    eprintln!(
        "This example opens and configures real HydraSDR hardware through the async API.\n\
         It is gated to avoid accidental USB access during checks/tests.\n\n\
         Run:\n\
           cargo run --features smol --example rx_async -- --run       # open/query/configure\n\
           cargo run --features smol --example rx_async -- --run --rx  # additionally read one RX block"
    );
}
