//! Direct HydraSDR RFOne API translation on top of `nusb`.
//!
//! This crate is in the direct C-to-Rust translation phase. Names and behavior
//! intentionally stay close to the C host driver so parity tests can trace each
//! Rust path back to the original `hydrasdr_*` API. A later phase should layer a
//! smaller, more idiomatic Rust wrapper over these direct bindings.
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
