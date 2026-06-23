# hydrasdr-rs

Direct Rust translation of the HydraSDR RFOne host driver, currently focused on C API parity and traceability rather than an idiomatic high-level Rust API.

This crate is in the direct C-to-Rust translation phase. Public names intentionally stay close to the C driver (`hydrasdr_open`, `hydrasdr_set_freq`, `hydrasdr_start_rx`, and friends) so behavior can be compared against `/home/basti/src/hydrasdr-host`. A later phase is expected to layer a smaller, more Rust-like wrapper on top of this direct API.

## Status

Implemented in this branch:

- USB discovery/open for HydraSDR RFOne VID/PID pairs.
- Synchronous control API for board/version/serial queries, samplerate and bandwidth configuration, gain control, RF port selection, GPIO, clockgen, RF frontend, SPI flash, packing, receiver mode, and short RX streaming.
- Executor-agnostic async counterparts for the direct API.
- No-hardware parity tests for constants, control-transfer packing, state handling, error mapping, and streaming loop behavior.
- Hardware-gated smoke tests and examples for real devices.

Not yet polished:

- This is not the final ergonomic Rust API.
- DSP/DDC conversion and a higher-level safe streaming abstraction are phase boundaries, not part of the current direct translation.
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

## Basic direct synchronous usage

```rust,no_run
use hydrasdr_rs::HydraSdr;
use hydrasdr_rs::types::SampleType;

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

The callback receives raw USB bytes plus C-parity sample-count metadata. It is intentionally not a final typed IQ sample abstraction yet.

## Async direct usage

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

See `examples/direct_sync.rs` and `examples/direct_async.rs` for longer real-device flows.

## Checks and tests

Default verification, no HydraSDR required:

```sh
cargo check
cargo test
cargo fmt --check
cargo doc --no-deps
cargo check --examples
cargo run --example direct_sync --
cargo run --example direct_async --
cargo check --examples --features tokio
cargo check --examples --features smol
```

Hardware-gated commands, run only with an RFOne connected and USB permissions in place:

```sh
cargo test --test hardware -- --ignored --nocapture
cargo run --example direct_sync -- --run
cargo run --example direct_sync -- --run --rx
cargo run --example direct_async -- --run
cargo run --example direct_async -- --run --rx
```

The parity coverage matrix is tracked in `docs/c-parity-test-matrix.md`.
