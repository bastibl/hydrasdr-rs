//! Error/status mapping for the HydraSDR API.

use core::fmt;

/// Status codes mirrored from the C driver.
///
/// Negative values intentionally match `hydrasdr_error` constants where present.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(i32)]
pub(crate) enum StatusCode {
    InvalidParam = -2,
    NotFound = -5,
    Busy = -6,
    Unsupported = -12,
    LibUsb = -1000,
    StreamingStopped = -1003,
    Other = -9999,
}

impl StatusCode {
    /// Return the C status/error macro name for this code.
    pub(crate) const fn name(self) -> &'static str {
        match self {
            Self::InvalidParam => "HYDRASDR_ERROR_INVALID_PARAM",
            Self::NotFound => "HYDRASDR_ERROR_NOT_FOUND",
            Self::Busy => "HYDRASDR_ERROR_BUSY",
            Self::Unsupported => "HYDRASDR_ERROR_UNSUPPORTED",
            Self::LibUsb => "HYDRASDR_ERROR_LIBUSB",
            Self::StreamingStopped => "HYDRASDR_ERROR_STREAMING_STOPPED",
            Self::Other => "HYDRASDR_ERROR_OTHER",
        }
    }
}

/// Error type used by the HydraSDR API.
///
/// USB errors preserve the backend message while exposing a stable high-level [`ErrorKind`].
#[derive(Debug)]
pub struct Error {
    repr: ErrorRepr,
}

#[derive(Debug)]
enum ErrorRepr {
    Status(StatusCode),
    Usb {
        status: StatusCode,
        message: String,
    },
    InvalidConfig {
        field: &'static str,
        reason: &'static str,
    },
    StreamClosed(&'static str),
}

/// Stable high-level error category.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ErrorKind {
    /// A provided configuration value failed validation.
    InvalidConfig,
    /// No matching HydraSDR device was found.
    NotFound,
    /// The device or USB resource is already in use.
    Busy,
    /// The requested operation is not supported by the backend or device.
    Unsupported,
    /// The USB backend returned an error.
    Usb,
    /// The stream was already stopped, finished, or otherwise closed.
    StreamClosed,
    /// Any other driver or backend error.
    Other,
}

impl Error {
    /// Build a high-level configuration validation error.
    pub(crate) const fn invalid_config(field: &'static str, reason: &'static str) -> Self {
        Self {
            repr: ErrorRepr::InvalidConfig { field, reason },
        }
    }

    /// Build a high-level stream lifecycle error.
    pub(crate) const fn stream_closed(reason: &'static str) -> Self {
        Self {
            repr: ErrorRepr::StreamClosed(reason),
        }
    }

    /// Build an error from an internal direct-driver status code.
    pub(crate) const fn status(status: StatusCode) -> Self {
        Self {
            repr: ErrorRepr::Status(status),
        }
    }

    /// Return the high-level error category.
    pub const fn kind(&self) -> ErrorKind {
        match self.status_code() {
            StatusCode::InvalidParam => ErrorKind::InvalidConfig,
            StatusCode::NotFound => ErrorKind::NotFound,
            StatusCode::Busy => ErrorKind::Busy,
            StatusCode::Unsupported => ErrorKind::Unsupported,
            StatusCode::LibUsb => ErrorKind::Usb,
            StatusCode::StreamingStopped => ErrorKind::StreamClosed,
            _ => ErrorKind::Other,
        }
    }

    /// Return the C-style status code represented by this error.
    pub(crate) const fn status_code(&self) -> StatusCode {
        match &self.repr {
            ErrorRepr::Status(code) => *code,
            ErrorRepr::Usb { status, .. } => *status,
            ErrorRepr::InvalidConfig { .. } => StatusCode::InvalidParam,
            ErrorRepr::StreamClosed(_) => StatusCode::StreamingStopped,
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.repr {
            ErrorRepr::Status(code) => f.write_str(code.name()),
            ErrorRepr::Usb { status, message } => write!(f, "{}: {message}", status.name()),
            ErrorRepr::InvalidConfig { field, reason } => {
                write!(f, "invalid configuration for {field}: {reason}")
            }
            ErrorRepr::StreamClosed(reason) => write!(f, "stream closed: {reason}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<StatusCode> for Error {
    fn from(value: StatusCode) -> Self {
        Self::status(value)
    }
}

impl From<nusb::Error> for Error {
    fn from(value: nusb::Error) -> Self {
        let status = match value.kind() {
            nusb::ErrorKind::Busy => StatusCode::Busy,
            nusb::ErrorKind::NotFound => StatusCode::NotFound,
            nusb::ErrorKind::Unsupported => StatusCode::Unsupported,
            nusb::ErrorKind::PermissionDenied
            | nusb::ErrorKind::Disconnected
            | nusb::ErrorKind::Other => StatusCode::LibUsb,
            _ => StatusCode::LibUsb,
        };
        Self {
            repr: ErrorRepr::Usb {
                status,
                message: value.to_string(),
            },
        }
    }
}

impl From<nusb::transfer::TransferError> for Error {
    fn from(value: nusb::transfer::TransferError) -> Self {
        let status = match value {
            nusb::transfer::TransferError::InvalidArgument => StatusCode::InvalidParam,
            nusb::transfer::TransferError::Disconnected
            | nusb::transfer::TransferError::Cancelled
            | nusb::transfer::TransferError::Stall
            | nusb::transfer::TransferError::Fault
            | nusb::transfer::TransferError::Unknown(_) => StatusCode::LibUsb,
        };
        Self {
            repr: ErrorRepr::Usb {
                status,
                message: value.to_string(),
            },
        }
    }
}

/// Crate result alias using [`Error`].
pub type Result<T> = core::result::Result<T, Error>;
