//! Constants copied from the C headers for the direct parity layer.

#![allow(dead_code)]

/// C `HYDRASDR_VERSION` string mirrored for crate parity checks.
pub const HYDRASDR_VERSION: &str = "1.1.2";
/// C `HYDRASDR_VER_MAJOR` value.
pub const HYDRASDR_VER_MAJOR: u32 = 1;
/// C `HYDRASDR_VER_MINOR` value.
pub const HYDRASDR_VER_MINOR: u32 = 1;
/// C `HYDRASDR_VER_REVISION` value.
pub const HYDRASDR_VER_REVISION: u32 = 2;

/// Sentinel matching `HYDRASDR_BANDWIDTH_AUTO`.
pub const HYDRASDR_BANDWIDTH_AUTO: u32 = u32::MAX;

/// Default control-transfer timeout in milliseconds, matching the C driver.
pub const CTRL_TIMEOUT_MS: u64 = 500;
/// Longer timeout used by C for flash-chip erase commands.
pub const CTRL_TIMEOUT_CHIP_ERASE_MS: u64 = 32_000;

/// Default raw USB buffer size used by the RFOne streaming path.
pub const DEFAULT_BUFFER_SIZE: usize = 262_144;
/// Packed-mode USB buffer size used by the RFOne streaming path.
pub const PACKED_BUFFER_SIZE: usize = 6_144 * 24;
/// C raw-buffer count constant retained for parity tests.
pub const RAW_BUFFER_COUNT: usize = 8;
/// Maximum samplerate count used by the C helpers.
pub const MAX_SUPPORTED_RATE_COUNT: usize = 100;

/// Encode a HydraSDR semantic version like the C `HYDRASDR_MAKE_VERSION` macro.
pub const fn make_version(major: u32, minor: u32, revision: u32) -> u32 {
    (major << 24) | (minor << 16) | revision
}

/// Numeric version assembled with [`make_version`].
pub const HYDRASDR_VERSION_NUM: u32 = make_version(
    HYDRASDR_VER_MAJOR,
    HYDRASDR_VER_MINOR,
    HYDRASDR_VER_REVISION,
);
