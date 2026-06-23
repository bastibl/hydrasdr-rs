//! Stable namespace for the low-level direct C-style HydraSDR API.
//!
//! The root crate also keeps the existing direct modules and top-level
//! [`HydraSdr`] re-export for compatibility. New documentation can point at
//! this module when users need the parity/debugging layer explicitly.

pub mod commands {
    pub use crate::commands::*;
}

pub mod constants {
    pub use crate::constants::*;
}

pub mod discovery {
    pub use crate::discovery::*;
}

pub mod errors {
    pub use crate::errors::*;
}

pub mod rfone {
    pub use crate::rfone::*;
}

pub mod streaming {
    pub use crate::streaming::*;
}

pub mod types {
    pub use crate::types::*;
}

pub mod usb {
    pub use crate::usb::*;
}

pub use crate::device::HydraSdr;
pub use crate::errors::{Error, Result, StatusCode};
