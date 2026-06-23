use std::env;

use futures_lite::future::block_on;
use hydrasdr_rs::commands::GainType;
use hydrasdr_rs::device::HydraSdr;
use hydrasdr_rs::discovery;
use hydrasdr_rs::streaming::Transfer;
use hydrasdr_rs::types::SampleType;

const EXAMPLE_FREQ_HZ: u64 = 100_000_000;
const FALLBACK_SAMPLE_RATE: u32 = 10_000_000;

fn main() -> hydrasdr_rs::Result<()> {
    let args: Vec<String> = env::args().skip(1).collect();
    if !args.iter().any(|arg| arg == "--run") {
        print_usage();
        return Ok(());
    }

    block_on(run(args))
}

async fn run(args: Vec<String>) -> hydrasdr_rs::Result<()> {
    let run_rx = args.iter().any(|arg| arg == "--rx");

    println!("HydraSDR devices visible over USB:");
    for device in discovery::list_devices_async().await? {
        println!(
            "  {:#06x}:{:#06x} {} serial={:?} product={:?}",
            device.vid, device.pid, device.description, device.serial, device.product_string
        );
    }

    let mut dev = HydraSdr::open_async().await?;

    let board_id = dev.board_id_read_async().await?;
    let firmware_version = dev.version_string_read_async().await?;
    let part_serial = dev.board_partid_serialno_read_async().await?;
    let info = dev.get_device_info_async().await?;

    println!("opened board: {:?} ({})", board_id, info.board_name);
    println!("firmware: {firmware_version}");
    println!("part id: {:08x?}", part_serial.part_id);
    println!("serial: {:08x?}", part_serial.serial_no);
    println!("features: 0x{:08x}", info.features);

    let sample_rates = dev.get_samplerates_async().await?;
    println!("sample rates: {sample_rates:?}");
    let sample_rate = sample_rates
        .first()
        .copied()
        .unwrap_or(FALLBACK_SAMPLE_RATE);

    dev.set_sample_type(SampleType::Raw)?;
    dev.set_freq_async(EXAMPLE_FREQ_HZ).await?;
    dev.set_samplerate_async(sample_rate).await?;
    dev.set_lna_gain_async(8).await?;
    dev.set_mixer_gain_async(8).await?;
    dev.set_vga_gain_async(8).await?;
    dev.set_gain_async(GainType::Linearity, 10).await?;

    println!("configured: freq={EXAMPLE_FREQ_HZ}Hz sample_rate={sample_rate} sample_type=Raw");

    if run_rx {
        let mut callbacks = 0;
        let stats = dev
            .start_rx_async(|transfer: &Transfer<'_>| {
                callbacks += 1;
                println!(
                    "rx buffer: {} bytes, {} samples, dropped={}",
                    transfer.samples.len(),
                    transfer.sample_count,
                    transfer.dropped_samples
                );
                1
            })
            .await?;
        println!("short async RX complete after {callbacks} callback(s): {stats:?}");
    } else {
        println!("RX not started; pass --rx with --run for a one-buffer async smoke stream.");
    }

    dev.close()
}

fn print_usage() {
    eprintln!(
        "This example opens and configures real HydraSDR hardware through the async API.\n\
         It is gated to avoid accidental USB access during checks/tests.\n\n\
         Run:\n\
           cargo run --example direct_async -- --run       # list/open/query/configure\n\
           cargo run --example direct_async -- --run --rx  # additionally read one RX buffer"
    );
}
