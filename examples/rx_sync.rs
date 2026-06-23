//! Synchronous HydraSDR receive example.
//!
//! The default invocation prints usage and exits without touching USB. Pass
//! `--run` only when an RFOne is connected and USB permissions are configured.

use std::env;

use hydrasdr_rs::{Device, GainPreset, RfPort, SampleFormat};

const EXAMPLE_FREQ_HZ: u64 = 100_000_000;
const EXAMPLE_SAMPLE_RATE_HZ: u32 = 10_000_000;

fn main() -> hydrasdr_rs::Result<()> {
    let args: Vec<String> = env::args().skip(1).collect();
    if !args.iter().any(|arg| arg == "--run") {
        print_usage();
        return Ok(());
    }

    let run_rx = args.iter().any(|arg| arg == "--rx");
    // DeviceBuilder validates the receiver configuration before opening and
    // applying it to the hardware.
    let mut dev = Device::builder()
        .frequency_hz(EXAMPLE_FREQ_HZ)
        .sample_rate_hz(EXAMPLE_SAMPLE_RATE_HZ)
        .sample_format(SampleFormat::RawAdc)
        .rf_port(RfPort::Rx0)
        .gain(GainPreset::Linearity(12))
        .bias_tee(false)
        .open()?;

    println!(
        "opened {} firmware={} features=0x{:08x}",
        dev.info().board_name,
        dev.info().firmware_version,
        dev.info().features
    );
    println!(
        "configured: freq={EXAMPLE_FREQ_HZ}Hz sample_rate={EXAMPLE_SAMPLE_RATE_HZ}Hz format=RawAdc"
    );

    if run_rx {
        let mut rx = dev.raw_rx_stream()?;
        if let Some(block) = rx.next_block()? {
            println!(
                "rx block: {} bytes, {} samples, dropped={}",
                block.raw_bytes().len(),
                block.sample_count(),
                block.dropped_samples()
            );
        }
        let stats = rx.finish()?;
        println!("short RX complete after one block: {stats:?}");
    } else {
        println!("RX not started; pass --rx with --run for a one-buffer smoke stream.");
    }

    Ok(())
}

fn print_usage() {
    eprintln!(
        "This example opens and configures real HydraSDR hardware.\n\
         It is gated to avoid accidental USB access during checks/tests.\n\n\
         Run:\n\
           cargo run --example rx_sync -- --run       # open/query/configure\n\
           cargo run --example rx_sync -- --run --rx  # additionally read one RX block"
    );
}
