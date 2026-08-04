//! Synchronous HydraSDR receive example.
//!
//! The default invocation prints usage and exits without touching USB. Pass
//! `--run` only when an RFOne is connected and USB permissions are configured.

use std::env;

use hydrasdr_rs::{Complex32, Device, GainPreset, MaybeFuture, RfPort};

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
    let dev = Device::builder()
        .frequency_hz(EXAMPLE_FREQ_HZ)
        .sample_rate_hz(EXAMPLE_SAMPLE_RATE_HZ)
        .rf_port(RfPort::Rx0)
        .gain(GainPreset::Linearity(12))
        .bias_tee(false)
        .open()
        .wait()?;

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
        let mut rx = dev.rx_stream()?;
        rx.start().wait()?;
        let mut samples = [Complex32::default(); 32];
        let count = rx
            .read(&mut samples, std::time::Duration::from_secs(1))
            .wait()?;
        let stats = rx.stop().wait()?;
        drop(rx);
        dev.shutdown().wait()?;
        println!("rx samples: {count}, first={:?}", samples.first());
        println!("short RX complete: {stats:?}");
    } else {
        println!("RX not started; pass --rx with --run for a one-buffer smoke stream.");
        dev.shutdown().wait()?;
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
