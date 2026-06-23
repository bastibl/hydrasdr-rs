//! Ergonomic async HydraSDR receive example.
//!
//! The default invocation prints usage and exits without touching USB. Pass
//! `--run` only when an RFOne is connected and USB permissions are configured.

use std::env;

#[cfg(any(feature = "smol", feature = "tokio"))]
use futures_lite::future::block_on;
#[cfg(any(feature = "smol", feature = "tokio"))]
use hydrasdr_rs::{Device, GainPreset, RfPort, SampleBlock, SampleFormat};

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
        return Ok(());
    }

    #[cfg(any(feature = "smol", feature = "tokio"))]
    block_on(run(args))
}

#[cfg(any(feature = "smol", feature = "tokio"))]
async fn run(args: Vec<String>) -> hydrasdr_rs::Result<()> {
    let run_rx = args.iter().any(|arg| arg == "--rx");
    // DeviceBuilder mirrors the synchronous API while staying
    // executor-agnostic at this crate layer.
    let mut dev = Device::builder()
        .frequency_hz(EXAMPLE_FREQ_HZ)
        .sample_rate_hz(EXAMPLE_SAMPLE_RATE_HZ)
        .sample_format(SampleFormat::RawU8Iq)
        .rf_port(RfPort::Rx0)
        .gain(GainPreset::Linearity(12))
        .bias_tee(false)
        .open_async()
        .await?;

    println!(
        "opened {} firmware={} features=0x{:08x}",
        dev.info().board_name,
        dev.info().firmware_version,
        dev.info().features
    );
    println!(
        "configured: freq={EXAMPLE_FREQ_HZ}Hz sample_rate={EXAMPLE_SAMPLE_RATE_HZ}Hz format=RawU8Iq"
    );

    if run_rx {
        let mut callbacks = 0;
        let stats = dev
            .receive_blocks_async(|block: SampleBlock<'_>| {
                callbacks += 1;
                println!(
                    "rx block: {} bytes, {} samples, dropped={}",
                    block.raw_bytes().len(),
                    block.sample_count(),
                    block.dropped_samples()
                );
                true // stop after the first callback for a short smoke stream
            })
            .await?;
        println!("short async RX complete after {callbacks} callback(s): {stats:?}");
    } else {
        println!("RX not started; pass --rx with --run for a one-buffer async smoke stream.");
    }

    Ok(())
}

fn print_usage() {
    eprintln!(
        "This ergonomic example opens and configures real HydraSDR hardware through the async API.\n\
         It is gated to avoid accidental USB access during checks/tests.\n\n\
         Run:\n\
           cargo run --features smol --example rx_async -- --run       # open/query/configure\n\
           cargo run --features smol --example rx_async -- --run --rx  # additionally read one RX block"
    );
}
