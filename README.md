# hydrasdr-rs

Rust HydraSDR RFOne access built on [`nusb`](https://crates.io/crates/nusb). The crate now leads with an ergonomic Rust API for normal receive workflows while keeping the direct C-style translation available for parity checks, debugging, and porting code from `hydrasdr-host`.

Use the high-level [`Device`](src/high_level.rs), [`DeviceBuilder`](src/high_level.rs), and [`Config`](src/config.rs) API for new Rust applications. Drop down to [`HydraSdr`](src/device.rs) or the explicit `hydrasdr_rs::direct` namespace when you need one-to-one access to the C-driver-shaped functions such as `set_freq`, `set_samplerate`, `start_rx`, or low-level GPIO/SPI/clockgen controls.

## Status

Implemented in this branch:

- Ergonomic sync and async device builders, reusable receiver `Config`, gain/sample/bandwidth selectors, direct escape hatches, and `SampleBlock` receive callbacks.
- USB discovery/open for HydraSDR RFOne VID/PID pairs.
- Synchronous control API for board/version/serial queries, samplerate and bandwidth configuration, gain control, RF port selection, GPIO, clockgen, RF frontend, SPI flash, packing, receiver mode, and short RX streaming.
- Executor-agnostic async counterparts for the direct API.
- No-hardware parity and ergonomic tests for constants, control-transfer packing, state handling, error mapping, configuration ordering, and streaming loop behavior.
- Persistent direct pull RX stream for unpacked `HYDRASDR_SAMPLE_FLOAT32_IQ`, including C-style virtual sample rates and DDC decimation.
- Hardware-gated smoke tests and sync/async examples for real devices.
- Design notes for the high-level API in `docs/ergonomic-api-design.md`.

Not yet polished:

- Packed-sample conversion and converted sample formats beyond pull-style `Float32Iq` are still phase boundaries; callback streaming intentionally exposes raw USB bytes.
- Async open/discovery and endpoint `clear_halt` follow what `nusb` exposes; this crate does not force a runtime by default.

## USB dependency and execution model

`hydrasdr-rs` uses [`nusb`](https://crates.io/crates/nusb) directly. It does not use libusb bindings.

The synchronous API calls `nusb::MaybeFuture::wait()` for discovery, device open, interface claim, control transfers, endpoint halt clearing, and bulk streaming completions. This is deliberate: the direct API mirrors the blocking shape of the C driver first, while keeping the USB backend in Rust.

The async API uses `nusb` futures for control and bulk transfers where available. Enable at most one async integration feature when the application needs a runtime-backed `nusb` IO thread:

```sh
cargo check --features tokio
cargo check --features smol
```

The default feature set stays runtime-free.

## Ergonomic synchronous usage

The high-level builder opens the selected RFOne, applies the receiver configuration through the direct API, and caches device metadata:

```rust,no_run
use hydrasdr_rs::commands::RfPort;
use hydrasdr_rs::{Device, GainPreset, SampleBlock, SampleFormat};

fn main() -> hydrasdr_rs::Result<()> {
    let mut dev = Device::builder()
        .frequency_hz(100_000_000)
        .sample_rate_hz(10_000_000)
        .sample_format(SampleFormat::RawU8Iq)
        .rf_port(RfPort::Rx0)
        .gain(GainPreset::Linearity(12))
        .bias_tee(false)
        .open()?;

    println!("opened {} ({})", dev.info().board_name, dev.info().firmware_version);

    let stats = dev.receive_blocks(|block: SampleBlock<'_>| {
        println!(
            "{} bytes, {} samples, dropped={}",
            block.raw_bytes().len(),
            block.sample_count(),
            block.dropped_samples()
        );
        true // stop after one callback
    })?;
    println!("{stats:?}");

    dev.into_direct().close()
}
```

For reusable validation without touching USB, build a `Config` first and pass it to an already-open device:

```rust
use hydrasdr_rs::{Config, GainPreset, SampleFormat};

let config = Config::builder()
    .frequency_hz(100_000_000)
    .sample_rate_hz(10_000_000)
    .sample_format(SampleFormat::RawU8Iq)
    .gain(GainPreset::Sensitivity(8))
    .build()?;

assert_eq!(config.frequency_hz(), 100_000_000);
# Ok::<(), hydrasdr_rs::Error>(())
```

## Ergonomic async usage

The async high-level API mirrors the sync shape and awaits the direct async control/streaming path. Enable exactly one runtime integration feature if your application needs `nusb`'s runtime-backed IO thread:

```rust,no_run
use futures_lite::future::block_on;
use hydrasdr_rs::commands::RfPort;
use hydrasdr_rs::{Device, GainPreset, SampleBlock, SampleFormat};

fn main() -> hydrasdr_rs::Result<()> {
    block_on(async {
        let mut dev = Device::builder()
            .frequency_hz(144_500_000)
            .sample_rate_hz(10_000_000)
            .sample_format(SampleFormat::RawU8Iq)
            .rf_port(RfPort::Rx0)
            .gain(GainPreset::Linearity(10))
            .open_async()
            .await?;

        let stats = dev.receive_blocks_async(|block: SampleBlock<'_>| {
            println!("async block: {} bytes", block.raw_bytes().len());
            true
        }).await?;
        println!("{stats:?}");

        dev.into_direct().close()
    })
}
```

See `examples/rx_sync.rs` and `examples/rx_async.rs` for hardware-gated ergonomic examples that are safe to compile without a connected RFOne and require `--run` before they touch USB.

## Linux permissions and hardware safety

Opening a real HydraSDR RFOne requires permission to access the USB device. On Linux this usually means either running as root for quick local experiments or installing an appropriate udev rule for the RFOne VID/PID pairs:

- legacy: `1d50:60a1`
- official: `38af:0001`

Example udev rule skeleton:

```udev
SUBSYSTEM=="usb", ATTR{idVendor}=="1d50", ATTR{idProduct}=="60a1", MODE="0660", GROUP="plugdev", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="38af", ATTR{idProduct}=="0001", MODE="0660", GROUP="plugdev", TAG+="uaccess"
```

Reload udev rules and replug the device before running hardware examples/tests. Exact group names vary by distribution.

Some API calls can change receiver state, RF bias, GPIO direction, or SPI flash contents. The examples and hardware tests are gated so normal `cargo test`/`cargo run --example ...` invocations do not accidentally touch hardware.

## Lower-level direct API

The direct API remains available for C-parity work and advanced controls that are not wrapped ergonomically yet. Use the top-level `HydraSdr` type or the explicit `hydrasdr_rs::direct` namespace:

```rust,no_run
use hydrasdr_rs::direct::HydraSdr;
use hydrasdr_rs::direct::types::SampleType;

fn main() -> hydrasdr_rs::Result<()> {
    let mut dev = HydraSdr::open()?;
    let info = dev.get_device_info()?;
    println!("firmware: {}", info.firmware_version);

    dev.set_sample_type(SampleType::Raw)?;
    dev.set_freq(100_000_000)?;
    dev.set_samplerate(10_000_000)?;

    dev.close()
}
```

Short direct RX example:

```rust,no_run
use hydrasdr_rs::HydraSdr;

fn main() -> hydrasdr_rs::Result<()> {
    let mut dev = HydraSdr::open()?;
    let stats = dev.start_rx(|transfer| {
        println!("{} bytes, {} samples", transfer.samples.len(), transfer.sample_count);
        1 // non-zero return stops streaming, matching the C callback contract
    })?;
    println!("{stats:?}");
    Ok(())
}
```

The callback receives raw USB bytes plus C-parity sample-count metadata. It intentionally mirrors the C driver rather than providing the higher-level pull-stream conversion.

Async direct usage is also available:

```rust,no_run
use futures_lite::future::block_on;
use hydrasdr_rs::HydraSdr;

fn main() -> hydrasdr_rs::Result<()> {
    block_on(async {
        let mut dev = HydraSdr::open_async().await?;
        let board = dev.board_id_read_async().await?;
        println!("board: {board:?}");
        dev.set_freq_async(100_000_000).await?;
        dev.close()
    })
}
```

See `examples/direct_sync.rs` and `examples/direct_async.rs` for longer real-device flows and for controls that intentionally stay close to the C API.

## Checks and tests

Default verification, no HydraSDR required:

```sh
cargo check
cargo test
cargo fmt --check
cargo doc --no-deps
cargo check --examples
cargo run --example rx_sync --
cargo run --example rx_async --
cargo run --example direct_sync --
cargo run --example direct_async --
cargo check --examples --features tokio
cargo check --examples --features smol
```

Hardware-gated commands, run only with an RFOne connected and USB permissions in place:

```sh
cargo test --test hardware -- --ignored --nocapture
cargo run --example rx_sync -- --run
cargo run --example rx_sync -- --run --rx
cargo run --example rx_async -- --run
cargo run --example rx_async -- --run --rx
cargo run --example direct_sync -- --run
cargo run --example direct_sync -- --run --rx
cargo run --example direct_async -- --run
cargo run --example direct_async -- --run --rx
```

The parity coverage matrix is tracked in `docs/c-parity-test-matrix.md`.
