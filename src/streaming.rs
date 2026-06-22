use crate::types::SampleType;

#[derive(Debug)]
pub struct Transfer<'a> {
    pub samples: &'a [u8],
    pub sample_count: i32,
    pub dropped_samples: u64,
    pub sample_type: SampleType,
}

pub type SampleBlockCallback = dyn FnMut(&Transfer<'_>) -> i32 + Send;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct StreamingStats {
    pub buffers_received: u64,
    pub buffers_processed: u64,
    pub buffers_dropped: u64,
}
