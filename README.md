# HydraSDR Rust Driver (WIP)

!!! Beware AI Slop !!!

This is mainly an experiment for using [`nusb`](https://crates.io/crates/nusb) for a real Rust-native driver that does *not* require `libusb`.
There are only a few Rust SDR drivers and the ones I know of are based on `rusb`, which wraps `libusb`.
Also `nusb`'s main interface is async, which fits well with [FutureSDR](https://github.com/futuresdr/futuresdr) and Seify's async API. `nusb` now supports WebUSB, so the async driver can use the same API on native and web targets.

Rust HydraSDR RFOne driver built on [`nusb`](https://crates.io/crates/nusb). The public API provides synchronous and asynchronous device, configuration, and receive-stream types on native targets; WebUSB builds expose only the async USB operations and owned async streams.

Use [`Device`](src/high_level.rs), [`DeviceBuilder`](src/high_level.rs), and [`Config`](src/config.rs) for applications.

## Status

Implemented:

- Sync and async device builders, reusable receiver `Config`, gain/sample/bandwidth selectors, and pull-style receive streams.
- USB discovery/open for HydraSDR RFOne VID/PID pairs, including WebUSB.
- Internal USB control implementation for board/version/serial queries, samplerate and bandwidth configuration, gain control, RF port selection, packing, receiver mode, and short RX streaming.
- Executor-agnostic async API counterparts.
- Complex float 32-bit sample conversion with device-reported rates and host-side decimation factors from 1x through 64x.
- Low-level raw ADC block streaming for applications that need raw USB blocks.

TODO:

- Packed-sample conversion.
- Type conversions other than complex float 32-bit.

## USB dependency and execution model

`hydrasdr-rs` uses [`nusb`](https://crates.io/crates/nusb) directly. It does not use libusb bindings.

One-shot operations implement `nusb::MaybeFuture`: call `.wait()` for synchronous use on native targets, or `.await` the same operation in async code. This covers discovery, device open, interface claim, control transfers, and configuration without duplicate `_async` methods.

The async API uses `nusb` futures for control and bulk transfers. This crate does not enforce an async runtime: by default, it has no runtime dependency and the synchronous API works without `tokio` or `smol`.

Receive streams are deliberately pull-based and do not spawn a background drain task. The default unpacked queue contains 16 transfers of 256 KiB (4 MiB total). Applications must keep `next_block` or `read` moving while RX is enabled; once that queue is exhausted, samples may be lost in the device. RFOne bulk transfers do not carry sequence numbers, so `StreamingStats` can count rejected host completions but cannot detect that device-side loss. Queue headroom is `4 MiB / raw USB byte rate` (about 105 ms at 20 Msamples/s with 16-bit unpacked samples).

For awaited USB operations on native targets, `nusb` needs runtime integration so it can run blocking OS work on an IO thread. Native applications that await `open`, `configure`, or owned async RX streams should normally enable one of this crate's forwarding features:

```sh
cargo check --features tokio
cargo check --features smol
```

Use `tokio` if the application already runs on Tokio; use `smol` for smaller examples or applications using the smol/async-io ecosystem.

## WebUSB

`wasm32-unknown-unknown` uses `nusb`'s WebUSB backend and only supports async USB operations. No `tokio` or `smol` feature is needed. Because `web-sys` still marks its WebUSB bindings as unstable, the final application crate must opt in. This repository includes the required target configuration in `.cargo/config.toml`:

```toml
[target.wasm32-unknown-unknown]
rustflags = ["--cfg=web_sys_unstable_apis"]
```

If `hydrasdr-rs` is consumed as a dependency, put the same target configuration in the application's Cargo configuration. Browser WebUSB access also requires a secure context, browser support, and user-granted permission for one of the RFOne VID/PID pairs. Call `Device::request_permission` from a transient user activation such as a click handler. `Device::list` and `Device::open` only operate on devices already authorized for the page, so opening works the same way in the browser window and in a Web Worker.

Check the WebUSB build with:

```sh
cargo check --target wasm32-unknown-unknown
```

## Configuration validation

`Config::builder()` and `Device::builder()` validate receiver settings before they touch hardware. RFOne center frequency must be `24_000_000..=1_800_000_000` Hz. Raw ADC sample rates must be `10_000..=65_535_999` Hz, while converted `F32Iq` sample rates must be `10_000..=32_767_999` Hz because the requested hardware rate is doubled before host-side IQ conversion. Manual bandwidths must be `1_000..=65_535_999` Hz.

Preset gains accept indexes `0..=21`. Manual RFOne gains accept LNA `0..=14`, mixer `0..=15`, and VGA `0..=15`.

## Synchronous API

The builder opens the selected RFOne, applies the receiver configuration, and caches device metadata:

```rust,no_run
use hydrasdr_rs::{Complex32, Device, GainPreset, MaybeFuture, RfPort, SampleFormat};

fn main() -> hydrasdr_rs::Result<()> {
    let dev = Device::builder()
        .frequency_hz(100_000_000)
        .sample_rate_hz(10_000_000)
        .sample_format(SampleFormat::F32Iq)
        .rf_port(RfPort::Rx0)
        .gain(GainPreset::Linearity(12))
        .bias_tee(false)
        .open()
        .wait()?;

    println!("opened {} ({})", dev.info().board_name, dev.info().firmware_version);

    let mut rx = dev.into_f32_rx_stream();
    rx.start()?;
    let mut samples = [Complex32::default(); 32];
    let count = rx.read(&mut samples, std::time::Duration::from_secs(1))?;
    println!("read {count} IQ samples");
    let stats = rx.stop()?;
    rx.shutdown().wait()?;
    println!("{stats:?}");

    Ok(())
}
```

## Asynchronous API

Async receive streams own the device and keep one USB transfer queue alive for their full lifetime. Enable exactly one runtime integration feature if your application needs `nusb`'s runtime-backed IO thread:

```rust,no_run
use futures_lite::future::block_on;
use hydrasdr_rs::{Complex32, Device, GainPreset, RfPort, SampleFormat};

fn main() -> hydrasdr_rs::Result<()> {
    block_on(async {
        let dev = Device::builder()
            .frequency_hz(144_500_000)
            .sample_rate_hz(10_000_000)
            .sample_format(SampleFormat::F32Iq)
            .rf_port(RfPort::Rx0)
            .gain(GainPreset::Linearity(10))
            .open()
            .await?;

        let mut rx = dev.into_async_f32_rx_stream();
        rx.start().await?;
        let mut samples = [Complex32::default(); 32];
        let count = rx.read(&mut samples).await?;
        println!("async samples: {count}");
        let stats = rx.stop().await?;
        rx.shutdown().await?;
        println!("{stats:?}");

        Ok(())
    })
}
```

Call `stop().await` to pause the receiver while retaining its transfer queue for restart; use `into_device()` afterward if the device handle is still needed. Call `shutdown().await` to consume the stream, close its queue, and explicitly turn off both reception and RF input bias power. Native drops run the same hardware shutdown sequence best-effort. WebUSB cannot issue asynchronous control transfers from `Drop`, so browser applications must await explicit shutdown. Because start and stop borrow the owned stream, canceling either future does not discard the device handle. `into_device()` remains available after receiver-off reports an error. Native backends cancel retained transfers at stop so they can be recycled quickly on restart. WebUSB has no cancellation primitive, so a restarted stream conservatively consumes and discards every submission that was pending at stop before it exposes new data; `StreamingStats::buffers_discarded_on_restart` reports that warm-up.

See `examples/rx_sync.rs` and `examples/rx_async.rs` for hardware-gated examples that are safe to compile without a connected RFOne and require `--run` before they touch USB.

## Linux permissions and hardware safety

Opening a real HydraSDR RFOne requires permission to access the USB device. On Linux, install an appropriate udev rule for the RFOne VID/PID pairs and make sure your user is a member of the `plugdev` group:

- legacy: `1d50:60a1`
- official: `38af:0001`

The default configuration disables RF input bias power. If an application enables it, explicitly await or wait for `shutdown()` before releasing the device. Native drops attempt receiver-off and bias-off cleanup as a safety net, but cannot report failures; WebUSB drops cannot issue asynchronous cleanup commands at all.

Example udev rule skeleton:

```udev
SUBSYSTEM=="usb", ATTR{idVendor}=="1d50", ATTR{idProduct}=="60a1", MODE="0660", GROUP="plugdev", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="38af", ATTR{idProduct}=="0001", MODE="0660", GROUP="plugdev", TAG+="uaccess"
```

Reload udev rules, replug the device, and start a new login session after adding your user to `plugdev`. Exact group names vary by distribution; if your system uses a different group, update the rule and group membership consistently.

Some API calls can change receiver state or RF bias. The examples and hardware tests are gated so normal `cargo test`/`cargo run --example ...` invocations do not accidentally touch hardware.

## Checks and tests

Hardware-gated commands, run only with an RFOne connected and USB permissions in place:

```sh
cargo test --test hardware -- --ignored --nocapture
cargo run --example rx_sync -- --run
cargo run --example rx_sync -- --run --rx
cargo run --features smol --example rx_async -- --run
cargo run --features smol --example rx_async -- --run --rx
```
