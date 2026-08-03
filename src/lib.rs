//! HydraSDR RFOne API on top of `nusb`.
//!
//! The crate exposes synchronous and asynchronous Rust APIs on native targets
//! and asynchronous USB I/O through WebUSB on `wasm32-unknown-unknown`.
//!
//! The synchronous API uses [`nusb::MaybeFuture::wait`] for blocking operation.
//! Async counterparts are executor-agnostic at this crate layer. Bulk/control
//! transfers are natively async in `nusb`; async device discovery/open and
//! endpoint `clear_halt` follow the support exposed by `nusb`, so applications
//! that await those paths can enable exactly one of this crate's `tokio` or
//! `smol` features for `nusb` IO thread integration. No async runtime is forced
//! on native targets. WebUSB does not need either runtime feature. No async
//! runtime is forced by default.
//!
//! WebUSB builds must enable `web_sys_unstable_apis` as described in the
//! [WebUSB section of the README](https://github.com/bastibl/hydrasdr-rs#webusb).
//! Browsers only expose devices for which the page has WebUSB permission;
//! `Device::open_async` requests it when necessary and must then be called from
//! a browser user gesture. Blocking USB methods and synchronous stream types are
//! not part of the `wasm32` API; use async device methods and owned async streams.
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

#![deny(missing_docs)]

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
pub use high_level::{AsyncF32RxStream, AsyncRawRxStream, Device, DeviceBuilder, SampleBlock};
#[cfg(not(target_arch = "wasm32"))]
pub use high_level::{F32RxStream, RawRxStream};
pub use streaming::StreamingStats;
pub use types::{BiasTeeInfo, DecimationMode, DeviceInfo, RfPortInfo};
