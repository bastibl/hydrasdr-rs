# hydrasdr-rs

Rust HydraSDR RFOne driver built on [`nusb`](https://crates.io/crates/nusb). The public API provides synchronous and asynchronous device, configuration, and sample-block types. The C-port remains inside the crate as the implementation and parity test bed.

Use [`Device`](src/high_level.rs), [`DeviceBuilder`](src/high_level.rs), and [`Config`](src/config.rs) for applications.

## Status

Implemented in this branch:

- Sync and async device builders, reusable receiver `Config`, gain/sample/bandwidth selectors, and `SampleBlock` receive callbacks.
- USB discovery/open for HydraSDR RFOne VID/PID pairs.
- Internal synchronous control implementation for board/version/serial queries, samplerate and bandwidth configuration, gain control, RF port selection, packing, receiver mode, and short RX streaming.
- Executor-agnostic async API counterparts.
- No-hardware public and internal parity tests for constants, control-transfer packing, state handling, error mapping, configuration ordering, and streaming loop behavior.
- Internal pull RX stream for unpacked `HYDRASDR_SAMPLE_FLOAT32_IQ`, including C-style virtual sample rates and DDC decimation.
- Hardware-gated smoke tests and sync/async examples for real devices.
- Design notes for the public API in `docs/ergonomic-api-design.md`.

Not yet polished:

- Packed-sample conversion and converted sample formats beyond pull-style `Float32Iq` are still phase boundaries; callback streaming intentionally exposes raw USB bytes.
- Async open/discovery and endpoint `clear_halt` follow what `nusb` exposes; this crate does not force a runtime by default.

## USB dependency and execution model

`hydrasdr-rs` uses [`nusb`](https://crates.io/crates/nusb) directly. It does not use libusb bindings.

The synchronous API calls `nusb::MaybeFuture::wait()` for discovery, device open, interface claim, control transfers, endpoint halt clearing, and bulk streaming completions while keeping the USB backend in Rust.

The async API uses `nusb` futures for control and bulk transfers where available. Enable at most one async integration feature when the application needs a runtime-backed `nusb` IO thread:

```sh
cargo check --features tokio
cargo check --features smol
```

The default feature set stays runtime-free.

## Synchronous API

The builder opens the selected RFOne, applies the receiver configuration, and caches device metadata:

```rust,no_run
use hydrasdr_rs::{Device, GainPreset, RfPort, SampleBlock, SampleFormat};

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

    Ok(())
}
```

## Asynchronous API

The async API mirrors the sync shape. Enable exactly one runtime integration feature if your application needs `nusb`'s runtime-backed IO thread:

```rust,no_run
use futures_lite::future::block_on;
use hydrasdr_rs::{Device, GainPreset, RfPort, SampleBlock, SampleFormat};

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

        Ok(())
    })
}
```

See `examples/rx_sync.rs` and `examples/rx_async.rs` for hardware-gated examples that are safe to compile without a connected RFOne and require `--run` before they touch USB.

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

Some API calls can change receiver state or RF bias. The examples and hardware tests are gated so normal `cargo test`/`cargo run --example ...` invocations do not accidentally touch hardware.

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
cargo check --examples --features tokio
cargo check --examples --features smol
```

Hardware-gated commands, run only with an RFOne connected and USB permissions in place:

```sh
cargo test --test hardware -- --ignored --nocapture
cargo run --example rx_sync -- --run
cargo run --example rx_sync -- --run --rx
cargo run --features smol --example rx_async -- --run
cargo run --features smol --example rx_async -- --run --rx
```

The parity coverage matrix is tracked in `docs/c-parity-test-matrix.md`.
