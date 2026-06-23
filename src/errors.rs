//! Error/status mapping for the direct HydraSDR API.

use core::fmt;

/// Status codes mirrored from the C driver.
///
/// Negative values intentionally match `hydrasdr_error` constants where present.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(i32)]
pub enum StatusCode {
    Success = 0,
    True = 1,
    InvalidParam = -2,
    NotFound = -5,
    Busy = -6,
    NoMem = -11,
    Unsupported = -12,
    LibUsb = -1000,
    Thread = -1001,
    StreamingThreadErr = -1002,
    StreamingStopped = -1003,
    Other = -9999,
}

impl StatusCode {
    /// Return the numeric C-compatible status code.
    pub const fn code(self) -> i32 {
        self as i32
    }

    /// Return the C status/error macro name for this code.
    pub const fn name(self) -> &'static str {
        match self {
            Self::Success => "HYDRASDR_SUCCESS",
            Self::True => "HYDRASDR_TRUE",
            Self::InvalidParam => "HYDRASDR_ERROR_INVALID_PARAM",
            Self::NotFound => "HYDRASDR_ERROR_NOT_FOUND",
            Self::Busy => "HYDRASDR_ERROR_BUSY",
            Self::NoMem => "HYDRASDR_ERROR_NO_MEM",
            Self::Unsupported => "HYDRASDR_ERROR_UNSUPPORTED",
            Self::LibUsb => "HYDRASDR_ERROR_LIBUSB",
            Self::Thread => "HYDRASDR_ERROR_THREAD",
            Self::StreamingThreadErr => "HYDRASDR_ERROR_STREAMING_THREAD_ERR",
            Self::StreamingStopped => "HYDRASDR_ERROR_STREAMING_STOPPED",
            Self::Other => "HYDRASDR_ERROR_OTHER",
        }
    }

    /// Return a C-style name for a raw status code, or the C fallback string for unknown values.
    pub fn name_for_code(code: i32) -> &'static str {
        Self::try_from(code).map_or("hydrasdr unknown error", Self::name)
    }
}

/// Raw status code that does not map to a known direct translation value.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct UnknownStatusCode(pub i32);

impl fmt::Display for UnknownStatusCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "unknown HydraSDR status code {}", self.0)
    }
}

impl std::error::Error for UnknownStatusCode {}

impl TryFrom<i32> for StatusCode {
    type Error = UnknownStatusCode;

    fn try_from(value: i32) -> core::result::Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Success),
            1 => Ok(Self::True),
            -2 => Ok(Self::InvalidParam),
            -5 => Ok(Self::NotFound),
            -6 => Ok(Self::Busy),
            -11 => Ok(Self::NoMem),
            -12 => Ok(Self::Unsupported),
            -1000 => Ok(Self::LibUsb),
            -1001 => Ok(Self::Thread),
            -1002 => Ok(Self::StreamingThreadErr),
            -1003 => Ok(Self::StreamingStopped),
            -9999 => Ok(Self::Other),
            other => Err(UnknownStatusCode(other)),
        }
    }
}

/// Return the C status/error macro name for a known status code.
pub fn error_name(code: StatusCode) -> &'static str {
    code.name()
}

/// Error type used by the direct API.
///
/// `nusb` errors are mapped to the closest C-style status code while preserving the USB message.
#[derive(Debug)]
pub enum Error {
    Status(StatusCode),
    UnknownStatus(UnknownStatusCode),
    Usb { status: StatusCode, message: String },
}

impl Error {
    /// Return the C-style status code represented by this error.
    pub const fn status_code(&self) -> StatusCode {
        match self {
            Self::Status(code) => *code,
            Self::UnknownStatus(_) => StatusCode::Other,
            Self::Usb { status, .. } => *status,
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Status(code) => f.write_str(code.name()),
            Self::UnknownStatus(code) => code.fmt(f),
            Self::Usb { status, message } => write!(f, "{}: {message}", status.name()),
        }
    }
}

impl std::error::Error for Error {}

impl From<StatusCode> for Error {
    fn from(value: StatusCode) -> Self {
        Self::Status(value)
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
        Self::Usb {
            status,
            message: value.to_string(),
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
        Self::Usb {
            status,
            message: value.to_string(),
        }
    }
}

/// Crate result alias using the direct API [`Error`].
pub type Result<T> = core::result::Result<T, Error>;
