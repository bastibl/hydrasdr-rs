//! HydraSDR RFOne API on top of `nusb`.
//!
//! One-shot USB operations implement [`MaybeFuture`]: await them in async code,
//! or call [`MaybeFuture::wait`] on native targets. Synchronous raw streams borrow
//! their device; converted synchronous streams and asynchronous streams own it.
//!
//! The synchronous API uses [`nusb::MaybeFuture::wait`] for blocking operation.
//! Awaited operations are executor-agnostic at this crate layer. Bulk/control
//! transfers are natively async in `nusb`; async device discovery/open and
//! endpoint `clear_halt` follow the support exposed by `nusb`, so applications
//! that await those paths can enable exactly one of this crate's `tokio` or
//! `smol` features for `nusb` IO thread integration. WebUSB does not need either
//! runtime feature. No async runtime is forced by default.
//!
//! WebUSB builds must enable `web_sys_unstable_apis` as described in the
//! [WebUSB section of the README](https://github.com/bastibl/hydrasdr-rs#webusb).
//! Browsers only expose devices for which the page has WebUSB permission. Call
//! `Device::request_permission` from a browser-window user gesture;
//! [`Device::open`] can then discover and open the authorized device from
//! either the window or a Web Worker. Opening never prompts for permission.
//! Blocking USB methods and synchronous stream types are not part of the
//! `wasm32` API; use async device methods and owned async streams.
//!
//! # Synchronous API
//!
//! Open and configure real hardware with the [`Device`] builder:
//!
//! ```no_run
//! use hydrasdr_rs::{Device, GainPreset, MaybeFuture, RfPort, SampleFormat};
//! use std::time::Duration;
//!
//! fn main() -> hydrasdr_rs::Result<()> {
//!     let mut dev = Device::builder()
//!         .frequency_hz(100_000_000)
//!         .sample_rate_hz(10_000_000)
//!         .sample_format(SampleFormat::RawAdc)
//!         .rf_port(RfPort::Rx0)
//!         .gain(GainPreset::Linearity(12))
//!         .open()
//!         .wait()?;
//!
//!     let mut rx = dev.raw_rx_stream()?;
//!     if let Some(block) = rx.next_block(Duration::from_secs(1))? {
//!         println!("{} bytes", block.raw_bytes().len());
//!     }
//!     let stats = rx.finish()?;
//!     println!("{stats:?}");
//!
//!     Ok(())
//! }
//! ```

#![deny(missing_docs)]

mod active_state;
mod commands;
mod config;
mod constants;
mod converter;
mod device;
mod discovery;
mod errors;
mod high_level;
mod maybe_future;
mod rfone;
mod streaming;
mod types;
mod usb;

pub use active_state::{ActiveState, GainStage};
pub use config::{Bandwidth, Config, ConfigBuilder, GainConfig, GainPreset, RfPort, SampleFormat};
pub use constants::MAX_F32_IQ_SAMPLES_PER_TRANSFER;
pub use discovery::DeviceDescriptor;
pub use errors::{Error, ErrorKind, Result};
pub use high_level::{AsyncF32RxStream, AsyncRawRxStream, Device, DeviceBuilder, SampleBlock};
#[cfg(not(target_arch = "wasm32"))]
pub use high_level::{F32RxStream, RawRxStream};
pub use nusb::MaybeFuture;
pub use streaming::StreamingStats;
pub use types::{BiasTeeInfo, DecimationMode, DeviceInfo, RfPortInfo};
