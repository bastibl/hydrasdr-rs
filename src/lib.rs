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

pub mod commands;
pub mod config;
pub mod constants;
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
