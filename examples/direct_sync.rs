use std::env;

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

    let run_rx = args.iter().any(|arg| arg == "--rx");

    println!("HydraSDR devices visible over USB:");
    for device in discovery::list_devices()? {
        println!(
            "  {:#06x}:{:#06x} {} serial={:?} product={:?}",
            device.vid, device.pid, device.description, device.serial, device.product_string
        );
    }

    let mut dev = HydraSdr::open()?;

    let board_id = dev.board_id_read()?;
    let firmware_version = dev.version_string_read()?;
    let part_serial = dev.board_partid_serialno_read()?;
    let info = dev.get_device_info()?;

    println!("opened board: {:?} ({})", board_id, info.board_name);
    println!("firmware: {firmware_version}");
    println!("part id: {:08x?}", part_serial.part_id);
    println!("serial: {:08x?}", part_serial.serial_no);
    println!("features: 0x{:08x}", info.features);

    let sample_rates = dev.get_samplerates()?;
    println!("sample rates: {sample_rates:?}");
    let sample_rate = sample_rates
        .first()
        .copied()
        .unwrap_or(FALLBACK_SAMPLE_RATE);

    dev.set_sample_type(SampleType::Raw)?;
    dev.set_freq(EXAMPLE_FREQ_HZ)?;
    dev.set_samplerate(sample_rate)?;
    dev.set_lna_gain(8)?;
    dev.set_mixer_gain(8)?;
    dev.set_vga_gain(8)?;
    dev.set_gain(GainType::Linearity, 10)?;

    println!("configured: freq={EXAMPLE_FREQ_HZ}Hz sample_rate={sample_rate} sample_type=Raw");

    if run_rx {
        let mut callbacks = 0;
        let stats = dev.start_rx(|transfer: &Transfer<'_>| {
            callbacks += 1;
            println!(
                "rx buffer: {} bytes, {} samples, dropped={}",
                transfer.samples.len(),
                transfer.sample_count,
                transfer.dropped_samples
            );
            1
        })?;
        println!("short RX complete after {callbacks} callback(s): {stats:?}");
    } else {
        println!("RX not started; pass --rx with --run for a one-buffer smoke stream.");
    }

    dev.close()
}

fn print_usage() {
    eprintln!(
        "This example opens and configures real HydraSDR hardware.\n\
         It is gated to avoid accidental USB access during checks/tests.\n\n\
         Run:\n\
           cargo run --example direct_sync -- --run       # list/open/query/configure\n\
           cargo run --example direct_sync -- --run --rx  # additionally read one RX buffer"
    );
}
