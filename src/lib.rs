//! Direct HydraSDR RFOne API translation on top of `nusb`.
//!
//! The synchronous API uses `nusb::MaybeFuture::wait()` to mirror the C driver.
//! Async counterparts are executor-agnostic at this crate layer and share the
//! same `VendorControlRequest` encoding and streaming state machinery as the
//! sync path. Bulk/control transfers are natively async in `nusb`; async device
//! discovery/open and endpoint `clear_halt` go through `nusb` blocking syscalls,
//! so applications that await those paths should enable exactly one of this
//! crate's `tokio` or `smol` features to select the corresponding `nusb` IO
//! thread integration. No async runtime is forced by default.

pub mod commands;
pub mod constants;
pub mod device;
pub mod discovery;
pub mod errors;
pub mod rfone;
pub mod streaming;
pub mod types;
pub mod usb;

pub use device::HydraSdr;
pub use errors::{Error, Result, StatusCode};
