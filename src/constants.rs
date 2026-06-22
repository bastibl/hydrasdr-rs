pub const HYDRASDR_VERSION: &str = "1.1.2";
pub const HYDRASDR_VER_MAJOR: u32 = 1;
pub const HYDRASDR_VER_MINOR: u32 = 1;
pub const HYDRASDR_VER_REVISION: u32 = 2;

pub const HYDRASDR_BANDWIDTH_AUTO: u32 = u32::MAX;

pub const CTRL_TIMEOUT_MS: u64 = 500;
pub const CTRL_TIMEOUT_CHIP_ERASE_MS: u64 = 32_000;

pub const DEFAULT_BUFFER_SIZE: usize = 262_144;
pub const PACKED_BUFFER_SIZE: usize = 6_144 * 24;
pub const RAW_BUFFER_COUNT: usize = 8;
pub const MAX_SUPPORTED_RATE_COUNT: usize = 100;

pub const fn make_version(major: u32, minor: u32, revision: u32) -> u32 {
    (major << 24) | (minor << 16) | revision
}

pub const HYDRASDR_VERSION_NUM: u32 = make_version(
    HYDRASDR_VER_MAJOR,
    HYDRASDR_VER_MINOR,
    HYDRASDR_VER_REVISION,
);
