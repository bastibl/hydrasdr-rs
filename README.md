# hydrasdr-rs

Rust HydraSDR RFOne driver built on [`nusb`](https://crates.io/crates/nusb). The public API provides synchronous and asynchronous device, configuration, and sample-block types. The C-port remains inside the crate as the implementation and parity test bed.

Use [`Device`](src/high_level.rs), [`DeviceBuilder`](src/high_level.rs), and [`Config`](src/config.rs) for applications.

## Status

Implemented:

- Sync and async device builders, reusable receiver `Config`, gain/sample/bandwidth selectors, and pull-style `SampleBlock` receive streams.
- USB discovery/open for HydraSDR RFOne VID/PID pairs.
- Internal synchronous control implementation for board/version/serial queries, samplerate and bandwidth configuration, gain control, RF port selection, packing, receiver mode, and short RX streaming.
- Executor-agnostic async API counterparts.
- No-hardware public and internal parity tests for constants, control-transfer packing, state handling, error mapping, configuration ordering, and streaming loop behavior.
- Internal pull RX stream for unpacked `HYDRASDR_SAMPLE_FLOAT32_IQ`, including C-style virtual sample rates and DDC decimation.
- Hardware-gated smoke tests and sync/async examples for real devices.

TODO:

- Packed-sample conversion.
- Public typed sample conversion APIs, such as `Float32Iq` blocks instead of raw USB bytes.

## USB dependency and execution model

`hydrasdr-rs` uses [`nusb`](https://crates.io/crates/nusb) directly. It does not use libusb bindings.

The synchronous API calls `nusb::MaybeFuture::wait()` for discovery, device open, interface claim, control transfers, endpoint halt clearing, and bulk streaming completions while keeping the USB backend in Rust.

The async API uses `nusb` futures for control and bulk transfers. This crate does not enforce an async runtime: by default, it has no runtime dependency and the synchronous API works without `tokio` or `smol`.

For async USB operations, `nusb` needs one runtime integration feature so it can run blocking OS work on an IO thread. Enable exactly one of this crate's forwarding features in applications that call `open_async`, `configure_async`, or `rx_stream_async`:

```sh
cargo check --features tokio
cargo check --features smol
```

Use `tokio` if the application already runs on Tokio; use `smol` for smaller examples or applications using the smol/async-io ecosystem. Do not enable both.

## Synchronous API

The builder opens the selected RFOne, applies the receiver configuration, and caches device metadata:

```rust,no_run
use hydrasdr_rs::{Device, GainPreset, RfPort, SampleFormat};

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

    let mut rx = dev.rx_stream()?;
    if let Some(block) = rx.next_block()? {
        println!(
            "{} bytes, {} samples, dropped={}",
            block.raw_bytes().len(),
            block.sample_count(),
            block.dropped_samples()
        );
    }
    let stats = rx.finish()?;
    println!("{stats:?}");

    Ok(())
}
```

## Asynchronous API

The async API mirrors the sync shape. Enable exactly one runtime integration feature if your application needs `nusb`'s runtime-backed IO thread:

```rust,no_run
use futures_lite::future::block_on;
use hydrasdr_rs::{Device, GainPreset, RfPort, SampleFormat};

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

        let mut rx = dev.rx_stream_async().await?;
        if let Some(block) = rx.next_block().await? {
            println!("async block: {} bytes", block.raw_bytes().len());
        }
        let stats = rx.finish().await?;
        println!("{stats:?}");

        Ok(())
    })
}
```

See `examples/rx_sync.rs` and `examples/rx_async.rs` for hardware-gated examples that are safe to compile without a connected RFOne and require `--run` before they touch USB.

## Linux permissions and hardware safety

Opening a real HydraSDR RFOne requires permission to access the USB device. On Linux, install an appropriate udev rule for the RFOne VID/PID pairs and make sure your user is a member of the `plugdev` group:

- legacy: `1d50:60a1`
- official: `38af:0001`

Example udev rule skeleton:

```udev
SUBSYSTEM=="usb", ATTR{idVendor}=="1d50", ATTR{idProduct}=="60a1", MODE="0660", GROUP="plugdev", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="38af", ATTR{idProduct}=="0001", MODE="0660", GROUP="plugdev", TAG+="uaccess"
```

Reload udev rules, replug the device, and start a new login session after adding your user to `plugdev`. Exact group names vary by distribution; if your system uses a different group, update the rule and group membership consistently.

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
