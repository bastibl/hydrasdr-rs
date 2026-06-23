#![allow(dead_code)]

//! HydraSDR RFOne API on top of `nusb`.
//!
//! The crate exposes synchronous and asynchronous Rust APIs for common receive
//! workflows. The low-level C-to-Rust port is kept inside the crate as the
//! USB/control implementation, but the public API is centered on [`Device`],
//! [`Config`], and typed configuration values.
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
//!         .sample_format(SampleFormat::RawU8Iq)
//!         .rf_port(RfPort::Rx0)
//!         .gain(GainPreset::Linearity(12))
//!         .open()?;
//!
//!     let mut rx = dev.rx_stream()?;
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

pub use commands::{GainType, RfPort};
pub use config::{
    Bandwidth, Config, ConfigBuilder, DeviceSelector, GainConfig, GainPreset, SampleFormat,
};
pub use errors::{Error, Result, StatusCode};
pub use high_level::{AsyncRxStream, Device, DeviceBuilder, RxStream, SampleBlock};
pub use streaming::StreamingStats;
pub use types::{
    BiasTeeInfo, BoardId, ComponentInfo, DeviceInfo, GainInfo, PartIdSerialNo, RfPortInfo,
    SampleType,
};

#[cfg(test)]
mod internal_tests {
    #[path = "async_api.rs"]
    mod async_api;
    #[path = "direct_api.rs"]
    mod direct_api;
    #[path = "ergonomic_api.rs"]
    mod ergonomic_api;
    #[path = "foundation.rs"]
    mod foundation;
    #[path = "no_hardware_parity.rs"]
    mod no_hardware_parity;
    #[path = "streaming.rs"]
    mod streaming;
}
