//! HydraSDR RFOne API on top of `nusb`.
//!
//! The crate exposes an ergonomic Rust layer for common receive workflows while
//! keeping the direct C-to-Rust translation available for parity/debugging. The
//! direct names remain available at their original top-level modules and under
//! the explicit [`direct`] namespace.
//!
//! The synchronous API uses [`nusb::MaybeFuture::wait`] to mirror the blocking C
//! driver. Async counterparts are executor-agnostic at this crate layer and
//! share the same `VendorControlRequest` encoding and streaming state machinery
//! as the sync path. Bulk/control transfers are natively async in `nusb`; async
//! device discovery/open and endpoint `clear_halt` follow the support exposed by
//! `nusb`, so applications that await those paths can enable exactly one of this
//! crate's `tokio` or `smol` features for `nusb` IO thread integration. No async
//! runtime is forced by default.
//!
//! Hardware access is never required for default tests. Real-device smoke tests
//! and examples are gated with `#[ignore]` or an explicit `--run` flag because
//! they open USB devices, change receiver state, and may touch RF bias/GPIO/SPI
//! flash paths just like the C driver.
//!
//! # Ergonomic API
//!
//! Build reusable receiver configurations without touching USB:
//!
//! ```
//! use hydrasdr_rs::{Config, GainPreset, SampleFormat};
//!
//! let config = Config::builder()
//!     .frequency_hz(100_000_000)
//!     .sample_rate_hz(10_000_000)
//!     .sample_format(SampleFormat::RawU8Iq)
//!     .gain(GainPreset::Sensitivity(8))
//!     .build()?;
//!
//! assert_eq!(config.frequency_hz(), 100_000_000);
//! # Ok::<(), hydrasdr_rs::Error>(())
//! ```
//!
//! Open and configure real hardware with the high-level [`Device`] builder:
//!
//! ```no_run
//! use hydrasdr_rs::commands::RfPort;
//! use hydrasdr_rs::{Device, GainPreset, SampleBlock, SampleFormat};
//!
//! fn main() -> hydrasdr_rs::Result<()> {
//!     let mut dev = Device::builder()
//!         .frequency_hz(100_000_000)
//!         .sample_rate_hz(10_000_000)
//!         .sample_format(SampleFormat::RawU8Iq)
//!         .rf_port(RfPort::Rx0)
//!         .gain(GainPreset::Linearity(12))
//!         .open()?;
//!
//!     let stats = dev.receive_blocks(|block: SampleBlock<'_>| {
//!         println!("{} bytes", block.raw_bytes().len());
//!         true
//!     })?;
//!     println!("{stats:?}");
//!
//!     dev.into_direct().close()
//! }
//! ```
//!
//! # Direct API
//!
//! The low-level C-style API is still available when you need one-to-one control
//! requests or parity with `hydrasdr-host`:
//!
//! ```no_run
//! use hydrasdr_rs::direct::HydraSdr;
//! use hydrasdr_rs::direct::types::SampleType;
//!
//! fn main() -> hydrasdr_rs::Result<()> {
//!     let mut dev = HydraSdr::open()?;
//!     dev.set_sample_type(SampleType::Raw)?;
//!     dev.set_freq(100_000_000)?;
//!     dev.set_samplerate(10_000_000)?;
//!     dev.close()
//! }
//! ```

pub mod commands;
pub mod config;
pub mod constants;
mod converter;
pub mod device;
pub mod direct;
pub mod discovery;
pub mod errors;
pub mod high_level;
pub mod rfone;
pub mod streaming;
pub mod types;
pub mod usb;

pub use config::{
    Bandwidth, Config, ConfigBuilder, DeviceSelector, GainConfig, GainPreset, SampleFormat,
};
pub use device::HydraSdr;
pub use errors::{Error, Result, StatusCode};
pub use high_level::{AsyncRxStream, Device, DeviceBuilder, RxStream, SampleBlock};
