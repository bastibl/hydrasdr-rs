//! Internal streaming and control constants.

/// Default control-transfer timeout in milliseconds, matching the C driver.
pub const CTRL_TIMEOUT_MS: u64 = 500;
/// Default raw USB buffer size used by the RFOne streaming path.
pub const DEFAULT_BUFFER_SIZE: usize = 262_144;
/// Packed-mode USB buffer size used by the RFOne streaming path.
pub const PACKED_BUFFER_SIZE: usize = 6_144 * 24;
