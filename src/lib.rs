//! HydraSDR RFOne API on top of `nusb`.
//!
//! The crate exposes synchronous and asynchronous Rust APIs.
//!
//! The synchronous API uses [`nusb::MaybeFuture::wait`] for blocking operation.
//! Async counterparts are executor-agnostic at this crate layer. Bulk/control
//! transfers are natively async in `nusb`; async device discovery/open and
//! endpoint `clear_halt` follow the support exposed by `nusb`, so applications
//! that await those paths can enable exactly one of this crate's `tokio` or
//! `smol` features for `nusb` IO thread integration. No async runtime is forced
//! by default.
//!
//! Hardware access is never required for default tests. Real-device smoke tests
//! and examples are gated with `#[ignore]` or an explicit `--run` flag because
//! they open USB devices, change receiver state, and may touch RF bias.
//!
//! # Synchronous API
//!
//! Open and configure real hardware with the [`Device`] builder:
//!
//! ```no_run
//! use hydrasdr_rs::{Device, GainPreset, RfPort, SampleFormat};
//!
//! fn main() -> hydrasdr_rs::Result<()> {
//!     let mut dev = Device::builder()
//!         .frequency_hz(100_000_000)
//!         .sample_rate_hz(10_000_000)
//!         .sample_format(SampleFormat::RawAdc)
//!         .rf_port(RfPort::Rx0)
//!         .gain(GainPreset::Linearity(12))
//!         .open()?;
//!
//!     let mut rx = dev.raw_rx_stream()?;
//!     if let Some(block) = rx.next_block()? {
//!         println!("{} bytes", block.raw_bytes().len());
//!     }
//!     let stats = rx.finish()?;
//!     println!("{stats:?}");
//!
//!     Ok(())
//! }
//! ```

mod commands;
mod config;
mod constants;
mod converter;
mod device;
mod discovery;
mod errors;
mod high_level;
mod rfone;
mod streaming;
mod types;
mod usb;

pub use config::{Bandwidth, Config, ConfigBuilder, GainConfig, GainPreset, RfPort, SampleFormat};
pub use discovery::DeviceDescriptor;
pub use errors::{Error, ErrorKind, Result};
pub use high_level::{
    AsyncF32RxStream, AsyncRawRxStream, Device, DeviceBuilder, F32RxStream, RawRxStream,
    SampleBlock,
};
pub use streaming::StreamingStats;
pub use types::{BiasTeeInfo, DecimationMode, DeviceInfo, RfPortInfo};
