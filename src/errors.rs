use core::fmt;

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
    pub const fn code(self) -> i32 {
        self as i32
    }

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

    pub fn name_for_code(code: i32) -> &'static str {
        Self::try_from(code).map_or("hydrasdr unknown error", Self::name)
    }
}

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

pub fn error_name(code: StatusCode) -> &'static str {
    code.name()
}

#[derive(Debug)]
pub enum Error {
    Status(StatusCode),
    UnknownStatus(UnknownStatusCode),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Status(code) => f.write_str(code.name()),
            Self::UnknownStatus(code) => code.fmt(f),
        }
    }
}

impl std::error::Error for Error {}

pub type Result<T> = core::result::Result<T, Error>;
