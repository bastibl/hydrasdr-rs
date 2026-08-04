//! Minimal RFOne ADC-to-IQ conversion used by direct streaming.
//!
//! This ports the unpacked 12-bit `float32_opt` path from `hydrasdr-host`:
//! LUT scaling, DC removal, Fs/4 polyphase DDC, and Q delay compensation.

#![allow(clippy::excessive_precision)]

use crate::Complex32;

const ADC_BITS: u32 = 12;
const ADC_MIDPOINT: f32 = (1_u32 << (ADC_BITS - 1)) as f32;
const DC_REMOVAL_ALPHA: f32 = 0.01;
const FIR_HISTORY_SIZE: usize = 512;
const FIR_MASK: usize = FIR_HISTORY_SIZE - 1;
const Q_DELAY_LEN: usize = 12;
const DEC_MAX_STAGES: usize = 6;
const DEC_STAGE_17_TAP_THRESHOLD: usize = 3;

const HB_KERNEL_FLOAT_33_FAST: [f32; 33] = [
    0.0,
    -0.0014766307,
    0.0,
    0.0037777218,
    0.0,
    -0.008223217,
    0.0,
    0.015783133,
    0.0,
    -0.028321939,
    0.0,
    0.050251268,
    0.0,
    -0.09755104,
    0.0,
    0.31537038,
    0.5,
    0.31537038,
    0.0,
    -0.09755104,
    0.0,
    0.050251268,
    0.0,
    -0.028321939,
    0.0,
    0.015783133,
    0.0,
    -0.008223217,
    0.0,
    0.0037777218,
    0.0,
    -0.0014766307,
    0.0,
];

const HB_KERNEL_FLOAT_17_FAST: [f32; 17] = [
    0.0,
    -0.006009383,
    0.0,
    0.025370203,
    0.0,
    -0.07765221,
    0.0,
    0.30773035,
    0.5,
    0.30773035,
    0.0,
    -0.07765221,
    0.0,
    0.025370203,
    0.0,
    -0.006009383,
    0.0,
];

const HB_KERNEL_FLOAT: [f32; 47] = [
    -0.0009986063,
    0.0,
    0.0016956373,
    0.0,
    -0.0030544302,
    0.0,
    0.0050555044,
    0.0,
    -0.007901319,
    0.0,
    0.011873357,
    0.0,
    -0.01741116,
    0.0,
    0.025304817,
    0.0,
    -0.037225224,
    0.0,
    0.057533287,
    0.0,
    -0.10232746,
    0.0,
    0.31703448,
    0.5,
    0.31703448,
    0.0,
    -0.10232746,
    0.0,
    0.057533287,
    0.0,
    -0.037225224,
    0.0,
    0.025304817,
    0.0,
    -0.01741116,
    0.0,
    0.011873357,
    0.0,
    -0.007901319,
    0.0,
    0.0050555044,
    0.0,
    -0.0030544302,
    0.0,
    0.0016956373,
    0.0,
    -0.0009986063,
];

#[derive(Debug)]
pub(crate) struct Float32IqConverter {
    fir_kernel: [f32; 24],
    fir_queue: [f32; FIR_HISTORY_SIZE * 2],
    delay_line: [f32; 16],
    decimation_stages: [DecimationStage; DEC_MAX_STAGES],
    scratch_a: Vec<f32>,
    scratch_b: Vec<f32>,
    avg: f32,
    fir_index: usize,
    delay_index: usize,
}

pub(crate) struct ConversionProgress {
    pub(crate) consumed_bytes: usize,
    pub(crate) written: usize,
    pub(crate) pending: Option<Complex32>,
}

impl Default for Float32IqConverter {
    fn default() -> Self {
        let mut fir_kernel = [0.0; 24];
        for (dst, src) in fir_kernel.iter_mut().zip(HB_KERNEL_FLOAT.iter().step_by(2)) {
            *dst = *src;
        }
        Self {
            fir_kernel,
            fir_queue: [0.0; FIR_HISTORY_SIZE * 2],
            delay_line: [0.0; 16],
            decimation_stages: core::array::from_fn(DecimationStage::new),
            scratch_a: Vec::new(),
            scratch_b: Vec::new(),
            avg: 0.0,
            fir_index: 0,
            delay_index: 0,
        }
    }
}

impl Float32IqConverter {
    #[cfg(test)]
    pub(crate) fn process_u16le_to_f32le(
        &mut self,
        raw: &[u8],
        decimation_factor: usize,
        out: &mut Vec<u8>,
    ) -> i32 {
        let decimation_factor = decimation_factor.max(1);
        let mut output = vec![Complex32::default(); (raw.len() / 2) / (2 * decimation_factor)];
        let progress = self.process_u16le_to_f32iq_slice(raw, decimation_factor, &mut output);
        debug_assert_eq!(progress.consumed_bytes, raw.len());
        debug_assert_eq!(progress.written, output.len());
        debug_assert!(progress.pending.is_none());

        out.reserve(output.len() * core::mem::size_of::<Complex32>());
        for value in output {
            out.extend_from_slice(&value.re.to_le_bytes());
            out.extend_from_slice(&value.im.to_le_bytes());
        }

        progress.written as i32
    }

    /// Convert a prefix of `raw` directly into `out`.
    ///
    /// The converter consumes input in chunks that preserve every decimation
    /// stage's four-sample phase. It can therefore produce one more complex
    /// sample than fits when `out` has odd length; that sample is returned for
    /// the stream to retain as fixed-size carry state.
    pub(crate) fn process_u16le_to_f32iq_slice(
        &mut self,
        raw: &[u8],
        decimation_factor: usize,
        out: &mut [Complex32],
    ) -> ConversionProgress {
        if out.is_empty() {
            return ConversionProgress {
                consumed_bytes: 0,
                written: 0,
                pending: None,
            };
        }

        let decimation_factor = decimation_factor.max(1);
        let raw_alignment = 4 * decimation_factor;
        let available_raw_samples = (raw.len() / 2) / raw_alignment * raw_alignment;
        let available_pairs = available_raw_samples / (2 * decimation_factor);
        let requested_pairs = out.len().saturating_add(1) & !1;
        let pairs = available_pairs.min(requested_pairs);
        if pairs == 0 {
            return ConversionProgress {
                consumed_bytes: 0,
                written: 0,
                pending: None,
            };
        }

        let raw_bytes = pairs * 2 * decimation_factor * 2;
        let written = out.len().min(pairs);
        let raw = &raw[..raw_bytes];
        let pending = if decimation_factor == 1 {
            self.process_baseband_to_complex(raw, &mut out[..written])
        } else {
            self.process_baseband_to_scratch(raw);
            let source_is_a = self.process_intermediate_decimation_stages(
                decimation_factor.trailing_zeros() as usize - 1,
            );
            let final_stage = decimation_factor.trailing_zeros() as usize - 1;
            if source_is_a {
                self.decimation_stages[final_stage]
                    .process_to_complex(&self.scratch_a, &mut out[..written])
            } else {
                self.decimation_stages[final_stage]
                    .process_to_complex(&self.scratch_b, &mut out[..written])
            }
        };
        debug_assert_eq!(pending.is_some(), written != pairs);

        ConversionProgress {
            consumed_bytes: raw_bytes,
            written,
            pending,
        }
    }

    fn process_baseband_to_scratch(&mut self, raw: &[u8]) {
        self.scratch_a.clear();
        self.scratch_a.reserve(raw.len() / 2);

        for chunk in raw.as_chunks::<8>().0 {
            let output = self.convert_adc_chunk(chunk);
            self.scratch_a.extend_from_slice(&output);
        }
    }

    fn process_baseband_to_complex(
        &mut self,
        raw: &[u8],
        out: &mut [Complex32],
    ) -> Option<Complex32> {
        let raw_chunks = raw.as_chunks::<8>().0;
        let (out_chunks, out_tail) = out.as_chunks_mut::<2>();
        debug_assert_eq!(
            raw_chunks.len(),
            out_chunks.len() + usize::from(!out_tail.is_empty())
        );

        for (raw_chunk, out_chunk) in raw_chunks.iter().zip(out_chunks) {
            let [i0, q0, i1, q1] = self.convert_adc_chunk(raw_chunk);
            *out_chunk = [Complex32::new(i0, q0), Complex32::new(i1, q1)];
        }

        out_tail.first_mut().map(|last| {
            let [i0, q0, i1, q1] = self.convert_adc_chunk(&raw_chunks[raw_chunks.len() - 1]);
            *last = Complex32::new(i0, q0);
            Complex32::new(i1, q1)
        })
    }

    fn convert_adc_chunk(&mut self, chunk: &[u8; 8]) -> [f32; 4] {
        let s0 = u16::from_le_bytes([chunk[0], chunk[1]]);
        let s1 = u16::from_le_bytes([chunk[2], chunk[3]]);
        let s2 = u16::from_le_bytes([chunk[4], chunk[5]]);
        let s3 = u16::from_le_bytes([chunk[6], chunk[7]]);

        let x0 = adc_to_float_12(s0) - self.avg;
        self.avg += DC_REMOVAL_ALPHA * x0;
        let x1 = adc_to_float_12(s1) - self.avg;
        self.avg += DC_REMOVAL_ALPHA * x1;
        let x2 = adc_to_float_12(s2) - self.avg;
        self.avg += DC_REMOVAL_ALPHA * x2;
        let x3 = adc_to_float_12(s3) - self.avg;
        self.avg += DC_REMOVAL_ALPHA * x3;

        let acc0 = self.push_fir(-x0);
        let acc1 = self.push_fir(x2);
        let q0 = self.push_delay(-x1 * 0.5);
        let q1 = self.push_delay(x3 * 0.5);

        [acc0, q0, acc1, q1]
    }

    fn push_fir(&mut self, value: f32) -> f32 {
        let idx = self.fir_index;
        self.fir_queue[idx] = value;
        self.fir_queue[idx + FIR_HISTORY_SIZE] = value;

        let src = idx + FIR_HISTORY_SIZE - self.fir_kernel.len() + 1;
        let mut acc = 0.0;
        for i in 0..12 {
            acc += self.fir_kernel[i]
                * (self.fir_queue[src + i] + self.fir_queue[src + self.fir_kernel.len() - 1 - i]);
        }

        self.fir_index = (self.fir_index + 1) & FIR_MASK;
        acc
    }

    fn push_delay(&mut self, value: f32) -> f32 {
        let out = self.delay_line[self.delay_index];
        self.delay_line[self.delay_index] = value;
        self.delay_index += 1;
        if self.delay_index >= Q_DELAY_LEN {
            self.delay_index = 0;
        }
        out
    }

    fn process_intermediate_decimation_stages(&mut self, num_stages: usize) -> bool {
        let mut source_is_a = true;
        for stage_idx in 0..num_stages {
            if source_is_a {
                self.scratch_b.clear();
                self.scratch_b.reserve(self.scratch_a.len() / 2);
                self.decimation_stages[stage_idx].process(&self.scratch_a, &mut self.scratch_b);
            } else {
                self.scratch_a.clear();
                self.scratch_a.reserve(self.scratch_b.len() / 2);
                self.decimation_stages[stage_idx].process(&self.scratch_b, &mut self.scratch_a);
            }
            source_is_a = !source_is_a;
        }
        source_is_a
    }
}

#[derive(Debug)]
struct DecimationStage {
    filter: DecimationFilter,
    queue_iq: Vec<f32>,
    fir_index: usize,
}

impl DecimationStage {
    fn new(stage_num: usize) -> Self {
        let filter = if stage_num < DEC_STAGE_17_TAP_THRESHOLD {
            DecimationFilter::Hb33
        } else {
            DecimationFilter::Hb17
        };
        Self {
            queue_iq: vec![0.0; filter.buf_size() * 2 * 2],
            filter,
            fir_index: 0,
        }
    }

    fn process(&mut self, src: &[f32], dest: &mut Vec<f32>) {
        self.process_outputs(src, |values| dest.extend_from_slice(&values));
    }

    fn process_to_complex(&mut self, src: &[f32], out: &mut [Complex32]) -> Option<Complex32> {
        let (out_chunks, out_tail) = out.as_chunks_mut::<2>();
        let mut out_chunks = out_chunks.iter_mut();
        let mut out_tail = out_tail.first_mut();
        let mut pending = None;
        self.process_outputs(src, |[i0, q0, i1, q1]| {
            if let Some(out_chunk) = out_chunks.next() {
                *out_chunk = [Complex32::new(i0, q0), Complex32::new(i1, q1)];
            } else {
                let last = out_tail.take().expect("decimator produced excess output");
                *last = Complex32::new(i0, q0);
                pending = Some(Complex32::new(i1, q1));
            }
        });
        debug_assert!(out_chunks.next().is_none());
        debug_assert!(out_tail.is_none());
        pending
    }

    fn process_outputs(&mut self, src: &[f32], mut emit: impl FnMut([f32; 4])) {
        match self.filter {
            DecimationFilter::Hb33 => self.process_hb33(src, &mut emit),
            DecimationFilter::Hb17 => self.process_hb17(src, &mut emit),
        }
    }

    fn process_hb33(&mut self, src: &[f32], emit: &mut impl FnMut([f32; 4])) {
        let pairs = (src.len() / 2) & !3;
        let mut idx = self.fir_index;
        for i in (0..pairs).step_by(4) {
            self.store_iq(idx, src[i * 2], src[i * 2 + 1]);
            idx = (idx + 1) & 31;

            self.store_iq(idx, src[i * 2 + 2], src[i * 2 + 3]);
            let idx0 = idx;
            idx = (idx + 1) & 31;

            self.store_iq(idx, src[i * 2 + 4], src[i * 2 + 5]);
            idx = (idx + 1) & 31;

            self.store_iq(idx, src[i * 2 + 6], src[i * 2 + 7]);
            let idx1 = idx;
            idx = (idx + 1) & 31;

            let (i0, q0) = self.acc_hb33(idx0);
            let (i1, q1) = self.acc_hb33(idx1);
            emit([i0, q0, i1, q1]);
        }
        self.fir_index = idx;
    }

    fn process_hb17(&mut self, src: &[f32], emit: &mut impl FnMut([f32; 4])) {
        let pairs = (src.len() / 2) & !3;
        let mut idx = self.fir_index;
        for i in (0..pairs).step_by(4) {
            self.store_iq(idx, src[i * 2], src[i * 2 + 1]);
            idx = (idx + 1) & 15;

            self.store_iq(idx, src[i * 2 + 2], src[i * 2 + 3]);
            let idx0 = idx;
            idx = (idx + 1) & 15;

            self.store_iq(idx, src[i * 2 + 4], src[i * 2 + 5]);
            idx = (idx + 1) & 15;

            self.store_iq(idx, src[i * 2 + 6], src[i * 2 + 7]);
            let idx1 = idx;
            idx = (idx + 1) & 15;

            let (i0, q0) = self.acc_hb17(idx0);
            let (i1, q1) = self.acc_hb17(idx1);
            emit([i0, q0, i1, q1]);
        }
        self.fir_index = idx;
    }

    fn store_iq(&mut self, idx: usize, i: f32, q: f32) {
        let buf_size = self.filter.buf_size();
        self.queue_iq[idx * 2] = i;
        self.queue_iq[idx * 2 + 1] = q;
        self.queue_iq[(idx + buf_size) * 2] = i;
        self.queue_iq[(idx + buf_size) * 2 + 1] = q;
    }

    fn acc_hb33(&self, idx: usize) -> (f32, f32) {
        let s = &self.queue_iq[(idx + 32 - 33 + 1) * 2..];
        let i = HB_KERNEL_FLOAT_33_FAST[1] * (s[2] + s[62])
            + HB_KERNEL_FLOAT_33_FAST[3] * (s[6] + s[58])
            + HB_KERNEL_FLOAT_33_FAST[5] * (s[10] + s[54])
            + HB_KERNEL_FLOAT_33_FAST[7] * (s[14] + s[50])
            + HB_KERNEL_FLOAT_33_FAST[9] * (s[18] + s[46])
            + HB_KERNEL_FLOAT_33_FAST[11] * (s[22] + s[42])
            + HB_KERNEL_FLOAT_33_FAST[13] * (s[26] + s[38])
            + HB_KERNEL_FLOAT_33_FAST[15] * (s[30] + s[34])
            + HB_KERNEL_FLOAT_33_FAST[16] * s[32];
        let q = HB_KERNEL_FLOAT_33_FAST[1] * (s[3] + s[63])
            + HB_KERNEL_FLOAT_33_FAST[3] * (s[7] + s[59])
            + HB_KERNEL_FLOAT_33_FAST[5] * (s[11] + s[55])
            + HB_KERNEL_FLOAT_33_FAST[7] * (s[15] + s[51])
            + HB_KERNEL_FLOAT_33_FAST[9] * (s[19] + s[47])
            + HB_KERNEL_FLOAT_33_FAST[11] * (s[23] + s[43])
            + HB_KERNEL_FLOAT_33_FAST[13] * (s[27] + s[39])
            + HB_KERNEL_FLOAT_33_FAST[15] * (s[31] + s[35])
            + HB_KERNEL_FLOAT_33_FAST[16] * s[33];
        (i, q)
    }

    fn acc_hb17(&self, idx: usize) -> (f32, f32) {
        let s = &self.queue_iq[(idx + 16 - 17 + 1) * 2..];
        let i = HB_KERNEL_FLOAT_17_FAST[1] * (s[2] + s[30])
            + HB_KERNEL_FLOAT_17_FAST[3] * (s[6] + s[26])
            + HB_KERNEL_FLOAT_17_FAST[5] * (s[10] + s[22])
            + HB_KERNEL_FLOAT_17_FAST[7] * (s[14] + s[18])
            + HB_KERNEL_FLOAT_17_FAST[8] * s[16];
        let q = HB_KERNEL_FLOAT_17_FAST[1] * (s[3] + s[31])
            + HB_KERNEL_FLOAT_17_FAST[3] * (s[7] + s[27])
            + HB_KERNEL_FLOAT_17_FAST[5] * (s[11] + s[23])
            + HB_KERNEL_FLOAT_17_FAST[7] * (s[15] + s[19])
            + HB_KERNEL_FLOAT_17_FAST[8] * s[17];
        (i, q)
    }
}

#[derive(Clone, Copy, Debug)]
enum DecimationFilter {
    Hb33,
    Hb17,
}

impl DecimationFilter {
    const fn buf_size(self) -> usize {
        match self {
            Self::Hb33 => 32,
            Self::Hb17 => 16,
        }
    }
}

fn adc_to_float_12(sample: u16) -> f32 {
    f32::from(sample & 0x0fff) / ADC_MIDPOINT - 1.0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn adc_bytes(samples: &[u16]) -> Vec<u8> {
        samples
            .iter()
            .flat_map(|sample| sample.to_le_bytes())
            .collect()
    }

    fn decode_f32(bytes: &[u8]) -> Vec<f32> {
        bytes
            .as_chunks::<4>()
            .0
            .iter()
            .map(|chunk| f32::from_le_bytes(*chunk))
            .collect()
    }

    fn assert_close(actual: f32, expected: f32) {
        assert!(
            (actual - expected).abs() <= 1e-5,
            "actual {actual:?} != expected {expected:?}"
        );
    }

    #[test]
    fn adc_lut_scaling_matches_c_formula() {
        assert_close(adc_to_float_12(0), -1.0);
        assert_close(adc_to_float_12(2048), 0.0);
        assert_close(adc_to_float_12(4095), 0.9995117);
    }

    #[test]
    fn float32_iq_golden_vector_matches_c_algorithm() {
        let input = adc_bytes(&[
            4095, 2048, 1024, 3072, 0, 100, 2000, 4095, 2048, 2049, 2050, 2051, 1024, 2048, 3072,
            4095,
        ]);
        let mut out = Vec::new();
        let mut converter = Float32IqConverter::default();

        let iq_pairs = converter.process_u16le_to_f32le(&input, 1, &mut out);
        let floats = decode_f32(&out);

        assert_eq!(iq_pairs, 8);
        assert_eq!(floats.len(), 16);
        let expected = [
            0.0009981188,
            0.0,
            -0.0011856249,
            0.0,
            0.0011800006,
            0.0,
            -0.0017698691,
            0.0,
            0.0022123493,
            0.0,
            -0.0026931474,
            0.0,
            0.0028032307,
            0.0,
            -0.0039764307,
            0.0,
        ];
        for (actual, expected) in floats.into_iter().zip(expected) {
            assert_close(actual, expected);
        }
    }

    #[test]
    fn converter_state_is_continuous_across_blocks() {
        let samples: Vec<_> = (0..64).map(|i| ((i * 73) % 4096) as u16).collect();
        let input = adc_bytes(&samples);

        let mut one_shot = Float32IqConverter::default();
        let mut expected = Vec::new();
        one_shot.process_u16le_to_f32le(&input, 1, &mut expected);

        let mut split = Float32IqConverter::default();
        let mut actual = Vec::new();
        split.process_u16le_to_f32le(&input[..48], 1, &mut actual);
        split.process_u16le_to_f32le(&input[48..], 1, &mut actual);

        assert_eq!(actual, expected);
    }

    #[test]
    fn sample_count_reports_output_iq_pairs() {
        let input = adc_bytes(&[2048; 16]);
        let mut out = Vec::new();
        let mut converter = Float32IqConverter::default();

        let iq_pairs = converter.process_u16le_to_f32le(&input, 1, &mut out);

        assert_eq!(iq_pairs, 8);
        assert_eq!(out.len(), 8 * 2 * core::mem::size_of::<f32>());
    }

    #[test]
    fn decimation_reduces_output_iq_pairs() {
        let samples: Vec<_> = (0..128).map(|i| ((i * 37) % 4096) as u16).collect();
        let input = adc_bytes(&samples);
        let mut out = Vec::new();
        let mut converter = Float32IqConverter::default();

        let iq_pairs = converter.process_u16le_to_f32le(&input, 4, &mut out);

        assert_eq!(iq_pairs, 16);
        assert_eq!(out.len(), 16 * 2 * core::mem::size_of::<f32>());
    }

    #[test]
    fn undecimated_slice_conversion_does_not_allocate_scratch_buffers() {
        let input = adc_bytes(&[2048; 128]);
        let mut output = [Complex32::default(); 64];
        let mut converter = Float32IqConverter::default();

        let progress = converter.process_u16le_to_f32iq_slice(&input, 1, &mut output);

        assert_eq!(progress.consumed_bytes, input.len());
        assert_eq!(progress.written, output.len());
        assert!(progress.pending.is_none());
        assert_eq!(converter.scratch_a.capacity(), 0);
        assert_eq!(converter.scratch_b.capacity(), 0);
    }

    #[test]
    fn final_decimation_stage_writes_without_an_output_scratch_buffer() {
        let input = adc_bytes(&[2048; 128]);
        let mut output = [Complex32::default(); 32];
        let mut converter = Float32IqConverter::default();

        let progress = converter.process_u16le_to_f32iq_slice(&input, 2, &mut output);

        assert_eq!(progress.consumed_bytes, input.len());
        assert_eq!(progress.written, output.len());
        assert!(progress.pending.is_none());
        assert_ne!(converter.scratch_a.capacity(), 0);
        assert_eq!(converter.scratch_b.capacity(), 0);
    }

    #[test]
    fn slice_conversion_matches_whole_transfer_for_all_decimations() {
        let samples: Vec<_> = (0..4096).map(|i| ((i * 37 + 11) % 4096) as u16).collect();
        let input = adc_bytes(&samples);

        for decimation in [1, 2, 4, 8, 16, 32, 64] {
            let mut whole = Float32IqConverter::default();
            let mut expected = vec![Complex32::default(); samples.len() / (2 * decimation)];
            let progress = whole.process_u16le_to_f32iq_slice(&input, decimation, &mut expected);
            assert_eq!(progress.consumed_bytes, input.len());
            assert_eq!(progress.written, expected.len());
            assert!(progress.pending.is_none());

            let mut sliced = Float32IqConverter::default();
            let mut actual = Vec::new();
            let mut offset = 0;
            let mut pending = None;
            let output_sizes = [1, 3, 8, 17];
            let mut call = 0;
            while offset != input.len() || pending.is_some() {
                let mut out = vec![Complex32::default(); output_sizes[call % output_sizes.len()]];
                call += 1;
                let mut written = 0;
                if let Some(sample) = pending.take() {
                    out[0] = sample;
                    written = 1;
                }
                if written != out.len() && offset != input.len() {
                    let progress = sliced.process_u16le_to_f32iq_slice(
                        &input[offset..],
                        decimation,
                        &mut out[written..],
                    );
                    assert_ne!(progress.consumed_bytes, 0);
                    offset += progress.consumed_bytes;
                    written += progress.written;
                    pending = progress.pending;
                }
                actual.extend_from_slice(&out[..written]);
            }

            assert_eq!(actual, expected, "decimation {decimation}");
        }
    }
}
