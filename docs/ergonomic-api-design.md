# Ergonomic Rust API design

This note proposes the later-stage public API layered on top of the verified direct C-style translation. The direct layer remains the parity/debugging escape hatch; the ergonomic layer should make the common RFOne receive path hard to misuse without hiding the low-level controls.

## Inputs from the direct translation

The current branch has a direct API that mirrors the C driver:

- `HydraSdr` is the direct device handle with sync and async control methods.
- `discovery::{list_devices, list_devices_async}` expose visible RFOne descriptors.
- `types::{DeviceInfo, SampleType, GainInfo, ...}` and `commands::{GainType, RfPort, ...}` mirror C enums/structs.
- `streaming::{Transfer, StreamingStats}` expose the C callback contract: raw USB bytes, C-parity sample counts, and non-zero callback return for stop.
- Default tests and examples are no-hardware safe; real-device flows are explicit/ignored.

The parent integration gate passed with the direct API committed on `port/direct-c-translation`. Known phase boundaries are: Android `open_fd`, decimation getter/setter wiring, and the C DSP/DDC conversion API.

## Public module shape

Add ergonomic modules without removing or renaming the direct API:

```text
hydrasdr_rs::Device             // high-level owned handle
hydrasdr_rs::DeviceBuilder      // open + initial config builder
hydrasdr_rs::Config             // reusable receiver configuration
hydrasdr_rs::RxStream           // synchronous active receive stream
hydrasdr_rs::AsyncRxStream      // async active receive stream
hydrasdr_rs::SampleBlock        // high-level sample block view
hydrasdr_rs::Error              // same crate error type, extended as needed
hydrasdr_rs::direct             // explicit low-level namespace, re-exporting direct API
```

Compatibility rule: the current direct API stays available. At minimum, keep the existing top-level direct names for the 0.1 line and also add `direct::{HydraSdr, commands, constants, discovery, errors, rfone, streaming, types, usb}` so new docs can point users at a stable direct namespace. The ergonomic layer must expose `Device::direct()`, `Device::direct_mut()`, and `Device::into_direct()` for parity debugging and unsupported operations.

## Builder and configuration types

`DeviceBuilder` should handle discovery/open selection and safe initial configuration:

```rust,ignore
let mut dev = hydrasdr_rs::Device::builder()
    .serial(0x0123_4567_89ab_cdef)
    .frequency_hz(100_000_000)
    .sample_rate_hz(10_000_000)
    .bandwidth_hz(10_000_000)
    .sample_format(SampleFormat::RawU8Iq)
    .rf_port(RfPort::Rx0)
    .gain(GainPreset::Linearity(12))
    .bias_tee(false)
    .open()?;
```

Proposed types:

- `DeviceSelector`: `First`, `Serial(u64)`, and later `UsbAddress { bus, address }` if `nusb` exposes enough stable identity.
- `Config`: frequency, sample rate, optional bandwidth (`Auto` by default), sample format, optional decimation mode, RF port, packing, gain plan, and bias tee state.
- `Bandwidth`: `Auto` or `ManualHz(u32)`. Preserve the C ordering rule by applying manual bandwidth before sample rate; for `Auto`, skip manual bandwidth and document direct-layer behavior.
- `SampleFormat`: high-level enum with conversion intent (`RawU8Iq`, `I16Iq`, `F32Iq`, real variants later). Initially map only to direct `SampleType` values that the current layer can deliver without DSP conversion.
- `GainPreset` / `GainConfig`: `Linearity(u8)`, `Sensitivity(u8)`, explicit component gains, and AGC toggles. Validate with `DeviceInfo` capabilities when available.

Builder methods should return `Self` for configuration and `Result<Device>` only at `open()`/`open_async()`. `Config::apply(&mut Device)` and `Config::apply_direct(&mut HydraSdr)` should be available so users can reconfigure an already-open device.

## `Device` ownership model

`Device` owns one direct `HydraSdr<NusbControl>` and a cached `DeviceInfo` after open/configuration. It should not be `Clone`. Keep mutable access for control operations so streaming and configuration cannot race in safe code.

Suggested methods:

```rust,ignore
impl Device {
    pub fn builder() -> DeviceBuilder;
    pub fn open() -> Result<Self>;
    pub fn open_serial(serial: u64) -> Result<Self>;
    pub async fn open_async() -> Result<Self>;
    pub async fn open_serial_async(serial: u64) -> Result<Self>;

    pub fn info(&self) -> &DeviceInfo;
    pub fn refresh_info(&mut self) -> Result<&DeviceInfo>;
    pub fn configure(&mut self, config: &Config) -> Result<()>;
    pub async fn configure_async(&mut self, config: &Config) -> Result<()>;

    pub fn rx_stream(&mut self) -> Result<RxStream<'_>>;
    pub async fn rx_stream_async(&mut self) -> Result<AsyncRxStream<'_>>;

    pub fn direct(&self) -> &HydraSdr;
    pub fn direct_mut(&mut self) -> &mut HydraSdr;
    pub fn into_direct(self) -> HydraSdr;
}
```

`Device` should apply the C-documented pre-streaming order internally: query capabilities/device info, set frequency, apply decimation once wired, apply manual bandwidth before sample rate, set sample rate, set sample format, then gains/RF port/bias/packing.

## Streaming ergonomics

The direct `start_rx` callback is useful for parity but awkward for Rust applications because it owns the whole receive loop and stops via an integer callback result. The ergonomic layer should expose an active stream object:

```rust,ignore
let mut stream = dev.rx_stream()?;
while let Some(block) = stream.next_block()? {
    process(block.samples());
    if done {
        stream.stop()?;
    }
}
let stats = stream.finish()?;
```

Recommended sync design:

- `RxStream<'dev>` borrows `&'dev mut Device`, so configuration cannot happen concurrently through safe references.
- `next_block()` returns `Result<Option<SampleBlock<'_>>>`; `Ok(None)` means an explicit stop/end condition, errors carry USB/status failures.
- `stop()` is idempotent and sends receiver off through the direct layer.
- `finish()` consumes the stream and returns `StreamingStats`, ensuring cancellation/receiver-off cleanup in `Drop` as a best-effort fallback.
- Initially, implement this by refactoring the direct streaming loop into reusable start/poll/stop primitives or by adding a channel-backed worker; prefer reusable direct primitives to avoid hidden threads in the sync API unless `nusb` requires it.

Recommended async design:

```rust,ignore
let mut stream = dev.rx_stream_async().await?;
while let Some(block) = stream.next_block().await? {
    process(block.samples());
}
```

- `AsyncRxStream<'dev>` mirrors `RxStream` with `async fn next_block(&mut self)` and `async fn stop(&mut self)`.
- Cancellation should be explicit and drop-safe: dropping the stream requests stop/cancels queued transfers and makes a best-effort receiver-off call where an async context is available. Because async `Drop` does not exist, document that `stop().await` or `finish().await` is required for guaranteed hardware cleanup.
- Keep callback helpers for users that prefer them: `Device::receive_blocks(|SampleBlock| ControlFlow<()>)` and `Device::receive_blocks_async(...)` can wrap the stream API.

## Sample representation

Start conservative:

```rust,ignore
pub struct SampleBlock<'a> {
    raw: &'a [u8],
    format: SampleFormat,
    dropped_samples: u64,
}
```

Expose `raw_bytes()` immediately. Add typed accessors only when they can be zero-copy and correct for the active format, for example `as_i8_iq()` and `as_i16_iq()` returning validated slice views or iterators. Do not promise float conversion/filtering until the C DSP/DDC conversion pipeline is ported. When conversion lands, gate it behind a feature and add owned/converted block types.

## Error model

Keep the existing `Error`/`Result` as the crate-wide type so direct and ergonomic users can interoperate. Extend it only if needed with high-level variants such as:

- `InvalidConfig { field, reason }`
- `UnsupportedConfig { field, capability }`
- `StreamClosed`
- `WouldBlock` only if a nonblocking polling API is added

Every high-level error should still expose a best-effort `status_code()` mapping for C-parity debugging.

## Feature flags

Keep `default = []` runtime-free. Existing `tokio` and `smol` features should continue to mean only `nusb` runtime integration.

Potential future flags:

- `stream`: high-level `Device`/`RxStream` API if the crate wants direct-only minimal builds. My recommendation: include high-level API by default because it adds little dependency surface.
- `convert`: DSP/DDC/sample conversion pipeline once ported from C.
- `unstable-direct-aliases`: not needed if `direct` is added as a permanent namespace; avoid it unless the public API churns.

Do not add a dependency on a specific async runtime beyond the existing `nusb` feature forwarding.

## Implementation plan

1. Add `src/direct.rs` re-exporting the existing direct modules/types and update crate docs to describe direct vs ergonomic layers. Keep current top-level direct exports for compatibility.
2. Add `src/config.rs` with `DeviceSelector`, `Bandwidth`, `SampleFormat`, `GainConfig`, `GainPreset`, and `Config` plus validation/apply-order unit tests using fake direct backends.
3. Add `src/high_level/device.rs` (or `src/device/high_level.rs` if the direct file is renamed later) with `Device` and `DeviceBuilder` wrapping `HydraSdr<NusbControl>`.
4. Refactor direct streaming internals enough to support an owned/polled stream object without changing C-parity tests. Keep `HydraSdr::start_rx` and `start_rx_async` as direct callback APIs.
5. Add `RxStream` and `AsyncRxStream` with explicit `stop`/`finish`, drop cleanup documentation, and no-hardware tests using fake streaming backends.
6. Add examples `examples/rx_sync.rs` and `examples/rx_async.rs` showing the ergonomic path; keep `direct_sync.rs` and `direct_async.rs` for parity/debugging.
7. Run the existing verification matrix (`fmt`, clippy all targets/features, tests, examples, docs) before committing each implementation slice.

## Open decisions

No blocker for implementation, but these should be kept visible in follow-up cards:

- Whether to keep `HydraSdr` top-level forever or move docs toward `direct::HydraSdr` while leaving a compatibility re-export.
- Whether `RxStream` should be internally polling or channel-backed if `nusb` makes reusable polling awkward.
- Exact naming of sample formats once the conversion pipeline exists (`SampleFormat` vs `SampleType` wrapper vs typed block traits).
