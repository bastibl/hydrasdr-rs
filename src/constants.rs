//! Internal streaming and control constants.

/// Default control-transfer timeout in milliseconds, matching the C driver.
pub(crate) const CTRL_TIMEOUT_MS: u64 = 500;
/// Default raw USB buffer size used by the RFOne streaming path.
pub(crate) const DEFAULT_BUFFER_SIZE: usize = 262_144;
/// Maximum number of unpacked complex `F32Iq` samples produced by one USB transfer.
///
/// Host-side decimation can make an individual transfer produce fewer samples.
pub const MAX_F32_IQ_SAMPLES_PER_TRANSFER: usize = DEFAULT_BUFFER_SIZE / (2 * size_of::<u16>());
/// Packed-mode USB buffer size used by the RFOne streaming path.
pub(crate) const PACKED_BUFFER_SIZE: usize = 6_144 * 24;
