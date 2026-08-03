use std::sync::Arc;

use nusb::MaybeFuture;

use crate::commands::{Capability, GainType, ReceiverMode, VendorRequest};
use crate::config::{Bandwidth, Config, RfPort};
use crate::discovery;
use crate::errors::{Error, Result};
use crate::maybe_future::{Either, MaybeFutureExt, ready};
use crate::rfone::{
    RFONE_HARDCODED_CAPS, RFONE_LINEARITY_LNA_GAINS, RFONE_LINEARITY_MIXER_GAINS,
    RFONE_LINEARITY_VGA_GAINS, RFONE_LNA_MAX_GAIN, RFONE_MAX_FREQ_HZ, RFONE_MIN_FREQ_HZ,
    RFONE_MIXER_MAX_GAIN, RFONE_RX_ENDPOINT, RFONE_SENSITIVITY_LNA_GAINS,
    RFONE_SENSITIVITY_MIXER_GAINS, RFONE_SENSITIVITY_VGA_GAINS, RFONE_VGA_MAX_GAIN,
    default_gain_infos, rf_port_infos,
};
use crate::streaming::{
    AsyncDirectRxStream, AsyncRawRxStream, AsyncStreamingBackend, StreamingState,
};
#[cfg(not(target_arch = "wasm32"))]
use crate::streaming::{DirectRxStream, RawRxStream, StreamingBackend, StreamingStats};
use crate::types::{BoardId, DecimationMode, DeviceInfo, GainInfo, PartIdSerialNo, SampleType};
use crate::usb::control::{
    ControlBackend, NusbControl, VendorControlRequest, decode_part_id_serial, decode_u32_le_words,
};

const EXPECTED_FW_PREFIX: &str = "HydraSDR RF";
const VERSION_STRING_SIZE: usize = 255;
const MIN_SAMPLERATE_BY_VALUE: u32 = 10_000;
const MIN_BANDWIDTH_BY_VALUE: u32 = 1_000;
const MAX_FREQ_HZ: u64 = 10_000_000_000;
const LEGACY_ADC_BITS: u8 = 12;
const DATA_FORMAT_RAW_ADC: u8 = 0;
const DECIMATION_FACTORS_ASC: [u32; 7] = [1, 2, 4, 8, 16, 32, 64];
const DECIMATION_FACTORS_DESC: [u32; 7] = [64, 32, 16, 8, 4, 2, 1];

/// Direct HydraSDR device handle.
///
/// The default backend is `nusb`; tests can inject fake control/streaming backends with
/// [`HydraSdr::from_control`] to verify C-parity request packing without hardware.
#[derive(Debug)]
pub(crate) struct HydraSdr<C = NusbControl> {
    control: Arc<C>,
    sample_type: SampleType,
    sample_rates: SampleRateTable,
    bandwidths: Vec<u32>,
    features: Option<u32>,
    gains: Vec<GainInfo>,
    current_samplerate: u32,
    hardware_samplerate: u32,
    decimation_factor: u32,
    decimation_mode: DecimationMode,
    current_bandwidth: u32,
    packing_enabled: bool,
    streaming: StreamingState,
}

struct AppliedConfig {
    sample_type: SampleType,
    decimation_mode: DecimationMode,
    bandwidth: Bandwidth,
    bandwidths: Vec<u32>,
    sample_rate: u32,
    rates: SampleRateTable,
    hardware_rate: u32,
    decimation: u32,
    packing: bool,
    gain_updates: Vec<GainUpdate>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct SampleRateInfo {
    rate_hz: u32,
    adc_bits: u8,
    data_format: u8,
    firmware_index: usize,
}

impl SampleRateInfo {
    fn legacy(rate_hz: u32, firmware_index: usize) -> Self {
        Self {
            rate_hz,
            adc_bits: LEGACY_ADC_BITS,
            data_format: DATA_FORMAT_RAW_ADC,
            firmware_index,
        }
    }

    fn supported_by_converter(self) -> bool {
        self.adc_bits == LEGACY_ADC_BITS && self.data_format == DATA_FORMAT_RAW_ADC
    }
}

#[derive(Clone, Debug)]
struct SampleRateTable {
    entries: Vec<SampleRateInfo>,
    extended: bool,
}

impl SampleRateTable {
    fn legacy(rates: Vec<u32>) -> Self {
        Self {
            entries: rates
                .into_iter()
                .enumerate()
                .map(|(index, rate_hz)| SampleRateInfo::legacy(rate_hz, index))
                .collect(),
            extended: false,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct SelectedSampleRate {
    param: u16,
    hardware_rate: u32,
    decimation: u32,
    response_len: usize,
    expected_encoding: Option<(u8, u8)>,
}

impl<C> HydraSdr<C> {
    /// Build a direct device handle from a control backend.
    pub(crate) fn from_control(control: C) -> Self {
        Self {
            control: Arc::new(control),
            sample_type: SampleType::Float32Iq,
            sample_rates: SampleRateTable::legacy(Vec::new()),
            bandwidths: Vec::new(),
            features: None,
            gains: default_gain_infos(),
            current_samplerate: 0,
            hardware_samplerate: 0,
            decimation_factor: 1,
            decimation_mode: DecimationMode::LowBandwidth,
            current_bandwidth: 0,
            packing_enabled: false,
            streaming: StreamingState::new(),
        }
    }

    /// Return the host-side decimation used by converted receive streams.
    pub(crate) fn streaming_decimation_factor(&self) -> usize {
        self.streaming.decimation_factor()
    }
}

impl<C: ControlBackend> HydraSdr<C> {
    /// Read the board ID, matching `hydrasdr_board_id_read`.
    pub(crate) fn board_id_read(&self) -> impl MaybeFuture<Output = Result<BoardId>> + use<C> {
        self.control_in_exact(VendorControlRequest::board_id_read(), 1)
            .map(|result| {
                let data = result?;
                BoardId::try_from(data[0]).map_err(|_| {
                    Error::protocol("read board ID", "firmware returned an unknown board ID")
                })
            })
    }

    /// Read the firmware version C string, matching `hydrasdr_version_string_read`.
    pub(crate) fn version_string_read(&self) -> impl MaybeFuture<Output = Result<String>> + use<C> {
        self.control_in_exact(
            VendorControlRequest::version_string_read(VERSION_STRING_SIZE),
            0,
        )
        .map(|result| result.map(|data| decode_c_string(&data)))
    }

    /// Read part ID and serial-number words, matching `hydrasdr_board_partid_serialno_read`.
    pub(crate) fn board_partid_serialno_read(
        &self,
    ) -> impl MaybeFuture<Output = Result<PartIdSerialNo>> + use<C> {
        self.control_in_exact(VendorControlRequest::board_partid_serialno_read(), 24)
            .map(|result| result.and_then(|data| decode_part_id_serial(&data)))
    }

    /// Read the primary firmware capability word.
    ///
    /// If firmware does not support the request, the RFOne hard-coded C capability mask is used.
    pub(crate) fn get_capabilities(&self) -> impl MaybeFuture<Output = Result<u32>> + use<C> {
        self.control_in_exact(VendorControlRequest::get_capabilities(0), 4)
            .map(|result| match result {
                Ok(data) => Ok(u32::from_le_bytes(
                    data[0..4].try_into().expect("four bytes"),
                )),
                Err(error) if is_unsupported_request(&error) => Ok(RFONE_HARDCODED_CAPS),
                Err(error) => Err(error),
            })
    }

    /// Read reserved capability words when firmware provides them.
    pub(crate) fn get_capabilities_reserved(
        &self,
    ) -> impl MaybeFuture<Output = Result<[u32; 3]>> + use<C> {
        let first = self.control_in_exact(VendorControlRequest::get_capabilities(1), 4);
        let second = self.control_in_exact(VendorControlRequest::get_capabilities(2), 4);
        let third = self.control_in_exact(VendorControlRequest::get_capabilities(3), 4);
        ready(Ok([0; 3]))
            .and_then(move |reserved| capability_word(first, reserved, 0))
            .and_then(move |reserved| capability_word(second, reserved, 1))
            .and_then(move |reserved| capability_word(third, reserved, 2))
    }

    /// Build direct device metadata from firmware queries and RFOne static tables.
    pub(crate) fn get_device_info(
        &mut self,
    ) -> impl MaybeFuture<Output = Result<DeviceInfo>> + use<'_, C> {
        let fetch = self.fetch_device_info();
        fetch.map(move |result| {
            let (info, features) = result?;
            self.features = Some(features);
            Ok(info)
        })
    }

    pub(crate) fn into_device_info(
        mut self,
    ) -> impl MaybeFuture<Output = Result<(Self, DeviceInfo)>> + use<C> {
        let fetch = self.fetch_device_info();
        fetch.map(move |result| {
            let (info, features) = result?;
            self.features = Some(features);
            Ok((self, info))
        })
    }

    fn fetch_device_info(&self) -> impl MaybeFuture<Output = Result<(DeviceInfo, u32)>> + use<C> {
        let board_id = self.board_id_read();
        let firmware = self.version_string_read();
        let part_serial = self.board_partid_serialno_read();
        let features = self.get_capabilities();
        let reserved = self.get_capabilities_reserved();
        board_id
            .map_err(|error| error.at("reading HydraSDR board ID"))
            .and_then(move |board_id| {
                firmware
                    .map_err(|error| error.at("reading HydraSDR firmware version"))
                    .map_ok(move |firmware| (board_id, firmware))
            })
            .and_then(move |(board_id, firmware)| {
                part_serial
                    .map_err(|error| error.at("reading HydraSDR part ID and serial number"))
                    .map_ok(move |part_serial| (board_id, firmware, part_serial))
            })
            .and_then(move |(board_id, firmware, part_serial)| {
                features.map_ok(move |features| (board_id, firmware, part_serial, features))
            })
            .and_then(move |(board_id, firmware, part_serial, features)| {
                reserved
                    .map_ok(move |reserved| (board_id, firmware, part_serial, features, reserved))
            })
            .map(move |result| {
                let (board_id, firmware, part_serial, features, reserved) = result?;
                Ok((
                    build_device_info(board_id, firmware, part_serial, features, reserved),
                    features,
                ))
            })
    }

    pub(crate) fn configure(
        &mut self,
        config: &Config,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let operation = self.prepare_config(config);
        operation.map(move |result| {
            self.apply_config_state(result?);
            Ok(())
        })
    }

    pub(crate) fn into_configured(
        mut self,
        config: Config,
    ) -> impl MaybeFuture<Output = Result<Self>> + use<C> {
        let operation = self.prepare_config(&config);
        operation.map(move |result| {
            self.apply_config_state(result?);
            Ok(self)
        })
    }

    fn prepare_config(
        &self,
        config: &Config,
    ) -> impl MaybeFuture<Output = Result<AppliedConfig>> + use<C> {
        let validation = config.validate();
        let frequency = config.frequency_hz();
        let sample_rate = config.sample_rate_hz();
        let sample_type = config.sample_format().sample_type();
        let decimation_mode = config.decimation_mode();
        let bandwidth = config.bandwidth();
        let port = config.rf_port();
        let gain = config.gain();
        let bias_tee = config.bias_tee();
        let packing = config.packing();

        let bandwidths = match bandwidth {
            Bandwidth::Auto => Either::left(ready(Ok(Vec::new()))),
            Bandwidth::ManualHz(_) => Either::right(self.available_bandwidths()),
        };
        let rates = self.available_samplerates();
        let control = Arc::clone(&self.control);
        let (gain_requests, gain_updates) = gain_config_plan(
            gain,
            self.features.unwrap_or(0) & Capability::ExtendedGain.bits() != 0,
        );
        let mut gain_requests = gain_requests.into_iter();
        let [gain0, gain1, gain2, gain3, gain4] = [
            gain_requests.next(),
            gain_requests.next(),
            gain_requests.next(),
            gain_requests.next(),
            gain_requests.next(),
        ];

        ready(validation)
            .and_then({
                let control = Arc::clone(&control);
                move |()| {
                    Self::control_out_with(control, VendorControlRequest::set_frequency(frequency))
                }
            })
            .and_then(move |()| bandwidths)
            .and_then({
                let control = Arc::clone(&control);
                move |bandwidths| {
                    let request = match bandwidth {
                        Bandwidth::Auto => Ok(None),
                        Bandwidth::ManualHz(hz) => bandwidth_param(&bandwidths, hz)
                            .map(|param| Some(VendorControlRequest::set_bandwidth(param))),
                    };
                    ready(request).and_then(move |request| {
                        optional_control_in(control, request).map_ok(move |_| bandwidths)
                    })
                }
            })
            .and_then(move |bandwidths| rates.map_ok(move |rates| (bandwidths, rates)))
            .and_then({
                let control = Arc::clone(&control);
                move |(bandwidths, rates)| {
                    let rate_config =
                        sample_rate_config(&rates, sample_type, decimation_mode, sample_rate);
                    ready(rate_config).and_then(move |selected| {
                        Self::control_in_exact_with(
                            control,
                            VendorControlRequest::set_samplerate(
                                selected.param,
                                selected.response_len,
                            ),
                            selected.response_len,
                        )
                        .map(move |result| {
                            validate_samplerate_response(&selected, &result?)?;
                            Ok((
                                bandwidths,
                                rates,
                                selected.hardware_rate,
                                selected.decimation,
                            ))
                        })
                    })
                }
            })
            .and_then({
                let control = Arc::clone(&control);
                move |state| {
                    optional_control_in(control, port.map(VendorControlRequest::set_rf_port)).map(
                        move |result| {
                            let response = result?;
                            if port.is_some() && response.first().copied() != Some(1) {
                                return Err(Error::protocol(
                                    "set RF port",
                                    "firmware rejected the requested RF port",
                                ));
                            }
                            Ok(state)
                        },
                    )
                }
            })
            .and_then({
                let control = Arc::clone(&control);
                move |state| gain_step(control, gain0).map_ok(move |_| state)
            })
            .and_then({
                let control = Arc::clone(&control);
                move |state| gain_step(control, gain1).map_ok(move |_| state)
            })
            .and_then({
                let control = Arc::clone(&control);
                move |state| gain_step(control, gain2).map_ok(move |_| state)
            })
            .and_then({
                let control = Arc::clone(&control);
                move |state| gain_step(control, gain3).map_ok(move |_| state)
            })
            .and_then({
                let control = Arc::clone(&control);
                move |state| gain_step(control, gain4).map_ok(move |_| state)
            })
            .and_then({
                let control = Arc::clone(&control);
                move |state| {
                    optional_control_out(
                        control,
                        bias_tee
                            .map(|enabled| VendorControlRequest::set_rf_bias(u8::from(enabled))),
                    )
                    .map_ok(move |_| state)
                }
            })
            .and_then(move |state| {
                Self::control_in_exact_with(
                    control,
                    VendorControlRequest::set_packing(u8::from(packing)),
                    1,
                )
                .map_ok(move |_| state)
            })
            .map(move |result| {
                let (bandwidths, rates, hardware_rate, decimation) = result?;
                Ok(AppliedConfig {
                    sample_type,
                    decimation_mode,
                    bandwidth,
                    bandwidths,
                    sample_rate,
                    rates,
                    hardware_rate,
                    decimation,
                    packing,
                    gain_updates,
                })
            })
    }

    fn apply_config_state(&mut self, state: AppliedConfig) {
        self.sample_type = state.sample_type;
        self.decimation_mode = state.decimation_mode;
        self.sample_rates = state.rates;
        if let Bandwidth::ManualHz(hz) = state.bandwidth {
            self.bandwidths = state.bandwidths;
            self.current_bandwidth = hz;
        }
        self.current_samplerate = state.sample_rate;
        self.hardware_samplerate = state.hardware_rate;
        self.decimation_factor = state.decimation;
        self.streaming
            .set_decimation(state.decimation as usize)
            .expect("validated HydraSDR decimation factor");
        self.packing_enabled = state.packing;
        self.streaming
            .set_packing(state.packing)
            .expect("validated HydraSDR packing state");
        for (gain_type, value, max_value) in state.gain_updates {
            self.update_gain_cache(gain_type, value, max_value);
        }
    }

    /// Read supported sample rates with the C count-then-list protocol.
    ///
    /// IQ sample modes return the C-style virtual rate table built from hardware rates and
    /// supported DDC decimation factors.
    pub(crate) fn get_samplerates(
        &mut self,
    ) -> impl MaybeFuture<Output = Result<Vec<u32>>> + use<'_, C> {
        let fetch = self.fetch_samplerates();
        fetch.map(move |result| {
            let table = result?;
            self.sample_rates = table;
            Ok(self.visible_sample_rates())
        })
    }

    /// Set sample rate by C-compatible index or kHz fallback calculation.
    pub(crate) fn set_samplerate(
        &mut self,
        samplerate: u32,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        self.set_samplerate_for_mode(samplerate, self.decimation_mode)
    }

    /// Read supported bandwidths with the C count-then-list protocol.
    pub(crate) fn get_bandwidths(
        &mut self,
    ) -> impl MaybeFuture<Output = Result<Vec<u32>>> + use<'_, C> {
        let fetch = self.available_bandwidths();
        fetch.map(move |result| {
            self.bandwidths = result?;
            Ok(self.bandwidths.clone())
        })
    }

    /// Set analog bandwidth by C-compatible index or kHz fallback calculation.
    pub(crate) fn set_bandwidth(
        &mut self,
        bandwidth: u32,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let bandwidths = self.available_bandwidths();
        let control = Arc::clone(&self.control);
        bandwidths
            .and_then(move |bandwidths| {
                let param = bandwidth_param(&bandwidths, bandwidth);
                ready(param).and_then(move |param| {
                    Self::control_in_exact_with(
                        control,
                        VendorControlRequest::set_bandwidth(param),
                        1,
                    )
                })
            })
            .map(move |result| {
                result?;
                self.current_bandwidth = bandwidth;
                Ok(())
            })
    }

    /// Set tuning frequency in Hz, matching `hydrasdr_set_freq` validation.
    pub(crate) fn set_freq(
        &mut self,
        freq_hz: u64,
    ) -> impl MaybeFuture<Output = Result<()>> + use<C> {
        let validation = if freq_hz == 0 || freq_hz > MAX_FREQ_HZ {
            Err(Error::invalid_config(
                "frequency_hz",
                "must be nonzero and at most 10 GHz",
            ))
        } else {
            Ok(())
        };
        let control = Arc::clone(&self.control);
        ready(validation).and_then(move |()| {
            Self::control_out_with(control, VendorControlRequest::set_frequency(freq_hz))
        })
    }

    pub(crate) fn set_gain_config(
        &mut self,
        gain: crate::GainConfig,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let (requests, updates) = gain_config_plan(
            gain,
            self.features.unwrap_or(0) & Capability::ExtendedGain.bits() != 0,
        );
        self.apply_gain_plan(requests, updates)
    }

    /// Select an RF input port and require the firmware success byte used by the C API.
    pub(crate) fn set_rf_port(
        &mut self,
        port: RfPort,
    ) -> impl MaybeFuture<Output = Result<()>> + use<C> {
        self.control_in_exact(VendorControlRequest::set_rf_port(port), 1)
            .map(|result| {
                let response = result?;
                if response.first().copied() != Some(1) {
                    return Err(Error::protocol(
                        "set RF port",
                        "firmware rejected the requested RF port",
                    ));
                }
                Ok(())
            })
    }

    /// Set receiver mode directly.
    pub(crate) fn receiver_mode(
        &self,
        mode: ReceiverMode,
    ) -> impl MaybeFuture<Output = Result<()>> + use<C> {
        self.control_out(VendorControlRequest::receiver_mode(mode))
    }

    fn update_gain_cache(&mut self, gain_type: GainType, value: u8, max_value: u8) {
        if let Some(gain) = self
            .gains
            .iter_mut()
            .find(|gain| gain.gain_type == gain_type)
        {
            gain.value = value;
            gain.max_value = gain.max_value.max(max_value);
            return;
        }
        self.gains.push(GainInfo {
            gain_type,
            min_value: 0,
            max_value,
            step_value: 1,
            default_value: value,
            value,
            flags: 0,
        });
    }

    fn set_samplerate_for_mode(
        &mut self,
        samplerate: u32,
        mode: DecimationMode,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let rates = self.available_samplerates();
        let control = Arc::clone(&self.control);
        let sample_type = self.sample_type;
        rates
            .and_then(move |rates| {
                let config = sample_rate_config(&rates, sample_type, mode, samplerate);
                ready(config).and_then(move |selected| {
                    Self::control_in_exact_with(
                        control,
                        VendorControlRequest::set_samplerate(selected.param, selected.response_len),
                        selected.response_len,
                    )
                    .map(move |result| {
                        validate_samplerate_response(&selected, &result?)?;
                        Ok((selected.hardware_rate, selected.decimation))
                    })
                })
            })
            .map(move |result| {
                let (hardware_rate, decimation) = result?;
                self.streaming.set_decimation(decimation as usize)?;
                self.current_samplerate = samplerate;
                self.hardware_samplerate = hardware_rate;
                self.decimation_factor = decimation;
                self.decimation_mode = mode;
                Ok(())
            })
    }

    fn fetch_samplerates(&self) -> impl MaybeFuture<Output = Result<SampleRateTable>> + use<C> {
        let control = Arc::clone(&self.control);
        let extended = self
            .features
            .is_some_and(|features| features & Capability::ExtendedSamplerates.bits() != 0);
        Self::control_in_exact_with(
            Arc::clone(&control),
            VendorControlRequest::get_samplerates_count(false),
            4,
        )
        .map(|result| {
            result.map(|data| u32::from_le_bytes(data[0..4].try_into().expect("four bytes")))
        })
        .and_then(move |count| {
            let extended_control = Arc::clone(&control);
            Self::control_in_exact_with(
                control,
                VendorControlRequest::get_samplerates(count, false),
                count as usize * 4,
            )
            .map(|result| result.and_then(|data| decode_u32_le_words(&data)))
            .and_then(move |rates| {
                if !extended {
                    return Either::left(ready(Ok(SampleRateTable::legacy(rates))));
                }

                let expected_len = count as usize * 8;
                Either::right(
                    Self::control_in_exact_with(
                        extended_control,
                        VendorControlRequest::get_samplerates(count, true),
                        expected_len,
                    )
                    .map(move |result| match result {
                        Ok(data) => decode_extended_samplerates(&data, &rates),
                        Err(error) if is_unsupported_request(&error) => {
                            Ok(SampleRateTable::legacy(rates))
                        }
                        Err(error) => Err(error),
                    }),
                )
            })
        })
    }

    fn available_samplerates(&self) -> impl MaybeFuture<Output = Result<SampleRateTable>> + use<C> {
        if self.sample_rates.entries.is_empty() {
            Either::left(self.fetch_samplerates())
        } else {
            Either::right(ready(Ok(self.sample_rates.clone())))
        }
    }

    fn fetch_bandwidths(&self) -> impl MaybeFuture<Output = Result<Vec<u32>>> + use<C> {
        let control = Arc::clone(&self.control);
        Self::control_in_exact_with(
            Arc::clone(&control),
            VendorControlRequest::get_bandwidths_count(),
            4,
        )
        .map(|result| {
            result.map(|data| u32::from_le_bytes(data[0..4].try_into().expect("four bytes")))
        })
        .and_then(move |count| {
            Self::control_in_exact_with(
                control,
                VendorControlRequest::get_bandwidths(count),
                count as usize * 4,
            )
            .map(|result| result.and_then(|data| decode_u32_le_words(&data)))
        })
    }

    fn available_bandwidths(&self) -> impl MaybeFuture<Output = Result<Vec<u32>>> + use<C> {
        if self
            .features
            .is_some_and(|features| features & Capability::Bandwidth.bits() == 0)
        {
            Either::left(ready(Err(Error::Unsupported)))
        } else {
            Either::right(if self.bandwidths.is_empty() {
                Either::left(self.fetch_bandwidths())
            } else {
                Either::right(ready(Ok(self.bandwidths.clone())))
            })
        }
    }

    fn apply_gain_plan(
        &mut self,
        requests: Vec<VendorControlRequest>,
        updates: Vec<(GainType, u8, u8)>,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let mut requests = requests.into_iter();
        let [step0, step1, step2, step3, step4] = [
            requests.next(),
            requests.next(),
            requests.next(),
            requests.next(),
            requests.next(),
        ];
        let control = Arc::clone(&self.control);
        ready(Ok(()))
            .and_then({
                let control = Arc::clone(&control);
                move |()| gain_step(control, step0)
            })
            .and_then({
                let control = Arc::clone(&control);
                move |()| gain_step(control, step1)
            })
            .and_then({
                let control = Arc::clone(&control);
                move |()| gain_step(control, step2)
            })
            .and_then({
                let control = Arc::clone(&control);
                move |()| gain_step(control, step3)
            })
            .and_then(move |()| gain_step(control, step4))
            .map(move |result| {
                result?;
                for (gain_type, value, max_value) in updates {
                    self.update_gain_cache(gain_type, value, max_value);
                }
                Ok(())
            })
    }

    fn control_in_exact(
        &self,
        request: VendorControlRequest,
        len: usize,
    ) -> impl MaybeFuture<Output = Result<Vec<u8>>> + use<C> {
        Self::control_in_exact_with(Arc::clone(&self.control), request, len)
    }

    fn control_in_exact_with(
        control: Arc<C>,
        request: VendorControlRequest,
        len: usize,
    ) -> impl MaybeFuture<Output = Result<Vec<u8>>> + use<C> {
        control.control_in(request).map(move |result| {
            let data = result?;
            if data.len() < len {
                return Err(Error::protocol(
                    "control transfer",
                    "response is shorter than requested",
                ));
            }
            Ok(data)
        })
    }

    fn control_out(
        &self,
        request: VendorControlRequest,
    ) -> impl MaybeFuture<Output = Result<()>> + use<C> {
        Self::control_out_with(Arc::clone(&self.control), request)
    }

    fn control_out_with(
        control: Arc<C>,
        request: VendorControlRequest,
    ) -> impl MaybeFuture<Output = Result<()>> + use<C> {
        control.control_out(request)
    }
}

impl<C> HydraSdr<C> {
    fn visible_sample_rates(&self) -> Vec<u32> {
        if !self.sample_type_is_iq() {
            return build_raw_samplerates(&self.sample_rates.entries);
        }
        build_virtual_samplerates(&self.sample_rates.entries)
    }

    fn sample_type_is_iq(&self) -> bool {
        self.sample_type == SampleType::Float32Iq
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl<C> HydraSdr<C>
where
    C: ControlBackend + StreamingBackend,
{
    /// Start a persistent synchronous pull RX stream for raw USB blocks.
    pub(crate) fn start_raw_rx_stream(&mut self) -> Result<RawRxStream<C::BulkIn>> {
        self.receiver_mode(ReceiverMode::Off).wait()?;
        let bulk_in = match self.control.as_ref().bulk_in(RFONE_RX_ENDPOINT) {
            Ok(bulk_in) => bulk_in,
            Err(err) => {
                let _ = self.receiver_mode(ReceiverMode::Off).wait();
                return Err(err);
            }
        };
        let prepared = RawRxStream::prepare(bulk_in, self.streaming.config());
        if let Err(err) = self.receiver_mode(ReceiverMode::Rx).wait() {
            let _ = self.receiver_mode(ReceiverMode::Off).wait();
            return Err(err);
        }

        match prepared.start_raw() {
            Ok(stream) => Ok(stream),
            Err(err) => {
                let _ = self.receiver_mode(ReceiverMode::Off).wait();
                Err(err)
            }
        }
    }

    /// Stop a persistent synchronous raw RX stream and return its accumulated counters.
    pub(crate) fn stop_raw_rx_stream(
        &mut self,
        stream: RawRxStream<C::BulkIn>,
    ) -> Result<StreamingStats> {
        let (stats, stop_result) = self.close_raw_rx_stream(stream);
        stop_result?;
        Ok(stats)
    }

    pub(crate) fn close_raw_rx_stream(
        &mut self,
        mut stream: RawRxStream<C::BulkIn>,
    ) -> (StreamingStats, Result<()>) {
        let stats = stream.close();
        (stats, self.receiver_mode(ReceiverMode::Off).wait())
    }

    /// Start a persistent synchronous pull RX stream for unpacked float32 IQ samples.
    pub(crate) fn start_rx_stream(&mut self) -> Result<DirectRxStream<C::BulkIn>> {
        if self.sample_type != SampleType::Float32Iq || self.packing_enabled {
            return Err(Error::Unsupported);
        }

        self.receiver_mode(ReceiverMode::Off).wait()?;
        let bulk_in = match self.control.as_ref().bulk_in(RFONE_RX_ENDPOINT) {
            Ok(bulk_in) => bulk_in,
            Err(err) => {
                let _ = self.receiver_mode(ReceiverMode::Off).wait();
                return Err(err);
            }
        };
        let prepared = DirectRxStream::prepare(bulk_in, self.streaming.config());
        if let Err(err) = self.receiver_mode(ReceiverMode::Rx).wait() {
            let _ = self.receiver_mode(ReceiverMode::Off).wait();
            return Err(err);
        }

        match prepared.start_direct() {
            Ok(stream) => Ok(stream),
            Err(err) => {
                let _ = self.receiver_mode(ReceiverMode::Off).wait();
                Err(err)
            }
        }
    }

    /// Stop a persistent synchronous pull RX stream and return its accumulated counters.
    pub(crate) fn stop_rx_stream(
        &mut self,
        stream: DirectRxStream<C::BulkIn>,
    ) -> Result<StreamingStats> {
        let (stats, stop_result) = self.close_rx_stream(stream);
        stop_result?;
        Ok(stats)
    }

    pub(crate) fn close_rx_stream(
        &mut self,
        mut stream: DirectRxStream<C::BulkIn>,
    ) -> (StreamingStats, Result<()>) {
        let stats = stream.close();
        (stats, self.receiver_mode(ReceiverMode::Off).wait())
    }
}

impl<C> HydraSdr<C>
where
    C: ControlBackend + AsyncStreamingBackend,
{
    /// Start a persistent async pull RX stream for raw USB blocks.
    pub(crate) async fn start_raw_rx_stream_async(
        &mut self,
    ) -> Result<AsyncRawRxStream<C::BulkIn>> {
        self.receiver_mode(ReceiverMode::Off).await?;
        let bulk_in = match self.control.as_ref().bulk_in_async(RFONE_RX_ENDPOINT).await {
            Ok(bulk_in) => bulk_in,
            Err(err) => {
                let _ = self.receiver_mode(ReceiverMode::Off).await;
                return Err(err);
            }
        };
        let prepared = AsyncRawRxStream::prepare(bulk_in, self.streaming.config());
        if let Err(err) = self.receiver_mode(ReceiverMode::Rx).await {
            let _ = self.receiver_mode(ReceiverMode::Off).await;
            return Err(err);
        }

        match prepared.start_async_raw().await {
            Ok(stream) => Ok(stream),
            Err(err) => {
                let _ = self.receiver_mode(ReceiverMode::Off).await;
                Err(err)
            }
        }
    }

    /// Start a persistent async pull RX stream for unpacked float32 IQ samples.
    pub(crate) async fn start_rx_stream_async(&mut self) -> Result<AsyncDirectRxStream<C::BulkIn>> {
        if self.sample_type != SampleType::Float32Iq || self.packing_enabled {
            return Err(Error::Unsupported);
        }

        self.receiver_mode(ReceiverMode::Off).await?;
        let bulk_in = match self.control.as_ref().bulk_in_async(RFONE_RX_ENDPOINT).await {
            Ok(bulk_in) => bulk_in,
            Err(err) => {
                let _ = self.receiver_mode(ReceiverMode::Off).await;
                return Err(err);
            }
        };
        let prepared = AsyncDirectRxStream::prepare(bulk_in, self.streaming.config());
        if let Err(err) = self.receiver_mode(ReceiverMode::Rx).await {
            let _ = self.receiver_mode(ReceiverMode::Off).await;
            return Err(err);
        }

        match prepared.start_async_direct().await {
            Ok(stream) => Ok(stream),
            Err(err) => {
                let _ = self.receiver_mode(ReceiverMode::Off).await;
                Err(err)
            }
        }
    }
}

impl HydraSdr<NusbControl> {
    /// Open the first visible HydraSDR RFOne through `nusb`, matching `hydrasdr_open`.
    pub(crate) fn open() -> impl MaybeFuture<Output = Result<Self>> {
        Self::open_selected(None)
    }

    /// Open a HydraSDR RFOne by parsed serial number, matching `hydrasdr_open_sn`.
    pub(crate) fn open_sn(serial: u64) -> impl MaybeFuture<Output = Result<Self>> {
        Self::open_selected(Some(serial))
    }

    fn open_selected(serial: Option<u64>) -> impl MaybeFuture<Output = Result<Self>> {
        discovery::select_nusb_device(serial)
            .and_then(|info| {
                info.open()
                    .map_err(Error::from)
                    .map_err(|error| error.at("opening USB device"))
            })
            .and_then(|device| {
                device.set_configuration(1).map(move |result| {
                    match result {
                        Ok(()) => {}
                        Err(err) if err.kind() == nusb::ErrorKind::Unsupported => {}
                        Err(err) => {
                            return Err(Error::from(err).at("selecting USB configuration 1"));
                        }
                    }
                    Ok(device)
                })
            })
            .and_then(|device| {
                device
                    .detach_and_claim_interface(0)
                    .map_err(Error::from)
                    .map_err(|error| error.at("claiming HydraSDR USB interface 0"))
                    .map_ok(move |interface| (device, interface))
            })
            .and_then(|(device, interface)| {
                let dev = Self::from_control(NusbControl::new(device, interface));
                dev.version_string_read()
                    .map_err(|error| error.at("validating HydraSDR firmware version"))
                    .map(move |result| {
                        let firmware = result?;
                        if !firmware.starts_with(EXPECTED_FW_PREFIX) {
                            return Err(Error::DeviceNotFound);
                        }
                        Ok(dev)
                    })
            })
    }
}

fn capability_word<F>(
    operation: F,
    mut reserved: [u32; 3],
    index: usize,
) -> impl MaybeFuture<Output = Result<[u32; 3]>>
where
    F: MaybeFuture<Output = Result<Vec<u8>>>,
{
    operation.map(move |result| match result {
        Ok(data) => {
            reserved[index] = u32::from_le_bytes(data[0..4].try_into().expect("four bytes"));
            Ok(reserved)
        }
        Err(error) if is_unsupported_request(&error) => Ok(reserved),
        Err(error) => Err(error),
    })
}

fn build_device_info(
    _board_id: BoardId,
    firmware_version: String,
    part_serial: PartIdSerialNo,
    _features: u32,
    _features_reserved: [u32; 3],
) -> DeviceInfo {
    DeviceInfo {
        serial: serial_from_part_id(&part_serial),
        board_name: "HydraSDR RFOne",
        firmware_version,
        min_frequency: RFONE_MIN_FREQ_HZ,
        max_frequency: RFONE_MAX_FREQ_HZ,
        rf_ports: rf_port_infos(),
        current_config: None,
    }
}

fn optional_control_in<C: ControlBackend>(
    control: Arc<C>,
    request: Option<VendorControlRequest>,
) -> impl MaybeFuture<Output = Result<Vec<u8>>> + use<C> {
    match request {
        Some(request) => Either::left(HydraSdr::<C>::control_in_exact_with(control, request, 1)),
        None => Either::right(ready(Ok(Vec::new()))),
    }
}

fn optional_control_out<C: ControlBackend>(
    control: Arc<C>,
    request: Option<VendorControlRequest>,
) -> impl MaybeFuture<Output = Result<()>> + use<C> {
    match request {
        Some(request) => Either::left(HydraSdr::<C>::control_out_with(control, request)),
        None => Either::right(ready(Ok(()))),
    }
}

fn gain_step<C: ControlBackend>(
    control: Arc<C>,
    request: Option<VendorControlRequest>,
) -> impl MaybeFuture<Output = Result<()>> + use<C> {
    match request {
        Some(request) => {
            Either::left(HydraSdr::<C>::control_in_exact_with(control, request, 1).map_ok(|_| ()))
        }
        None => Either::right(ready(Ok(()))),
    }
}

type GainUpdate = (GainType, u8, u8);

fn gain_config_plan(
    gain: crate::GainConfig,
    extended: bool,
) -> (Vec<VendorControlRequest>, Vec<GainUpdate>) {
    match gain {
        crate::GainConfig::Unchanged => (Vec::new(), Vec::new()),
        crate::GainConfig::Preset(crate::GainPreset::Linearity(value)) => {
            if extended {
                extended_gain_plan(GainType::Linearity, value)
            } else {
                gain_plan(GainType::Linearity, value)
            }
        }
        crate::GainConfig::Preset(crate::GainPreset::Sensitivity(value)) => {
            if extended {
                extended_gain_plan(GainType::Sensitivity, value)
            } else {
                gain_plan(GainType::Sensitivity, value)
            }
        }
        crate::GainConfig::Manual {
            lna,
            mixer,
            vga,
            lna_agc,
            mixer_agc,
        } if extended => {
            let mut requests = Vec::new();
            let mut updates = Vec::new();
            let mut push = |gain_type, value, max_value| {
                requests.push(VendorControlRequest::unified_gain(gain_type, value));
                updates.push((gain_type, value, max_value));
            };
            if let Some(value) = lna {
                push(GainType::Lna, value, RFONE_LNA_MAX_GAIN);
            }
            if let Some(value) = mixer {
                push(GainType::Mixer, value, RFONE_MIXER_MAX_GAIN);
            }
            if let Some(value) = vga {
                push(GainType::Vga, value, RFONE_VGA_MAX_GAIN);
            }
            if let Some(enabled) = lna_agc {
                push(GainType::LnaAgc, u8::from(enabled), 1);
            }
            if let Some(enabled) = mixer_agc {
                push(GainType::MixerAgc, u8::from(enabled), 1);
            }
            (requests, updates)
        }
        crate::GainConfig::Manual {
            lna,
            mixer,
            vga,
            lna_agc,
            mixer_agc,
        } => manual_gain_plan(lna, mixer, vga, lna_agc, mixer_agc),
    }
}

fn extended_gain_plan(
    gain_type: GainType,
    value: u8,
) -> (Vec<VendorControlRequest>, Vec<GainUpdate>) {
    (
        vec![VendorControlRequest::unified_gain(gain_type, value)],
        vec![(gain_type, value, value.max(1))],
    )
}

fn manual_gain_plan(
    lna: Option<u8>,
    mixer: Option<u8>,
    vga: Option<u8>,
    lna_agc: Option<bool>,
    mixer_agc: Option<bool>,
) -> (Vec<VendorControlRequest>, Vec<GainUpdate>) {
    let mut requests = Vec::new();
    let mut updates = Vec::new();
    let mut push = |gain_type, request, value, max_value| {
        requests.push(VendorControlRequest::legacy_gain(request, value));
        updates.push((gain_type, value, max_value));
    };
    if let Some(value) = lna {
        push(
            GainType::Lna,
            VendorRequest::SetLnaGain,
            value,
            RFONE_LNA_MAX_GAIN,
        );
    }
    if let Some(value) = mixer {
        push(
            GainType::Mixer,
            VendorRequest::SetMixerGain,
            value,
            RFONE_MIXER_MAX_GAIN,
        );
    }
    if let Some(value) = vga {
        push(
            GainType::Vga,
            VendorRequest::SetVgaGain,
            value,
            RFONE_VGA_MAX_GAIN,
        );
    }
    if let Some(enabled) = lna_agc {
        push(
            GainType::LnaAgc,
            VendorRequest::SetLnaAgc,
            u8::from(enabled),
            1,
        );
    }
    if let Some(enabled) = mixer_agc {
        push(
            GainType::MixerAgc,
            VendorRequest::SetMixerAgc,
            u8::from(enabled),
            1,
        );
    }
    (requests, updates)
}

fn gain_plan(gain_type: GainType, value: u8) -> (Vec<VendorControlRequest>, Vec<GainUpdate>) {
    let legacy = |gain_type, request, max_value| {
        let value = value.min(max_value);
        (
            vec![VendorControlRequest::legacy_gain(request, value)],
            vec![(gain_type, value, max_value)],
        )
    };
    match gain_type {
        GainType::Lna => legacy(GainType::Lna, VendorRequest::SetLnaGain, RFONE_LNA_MAX_GAIN),
        GainType::Mixer => legacy(
            GainType::Mixer,
            VendorRequest::SetMixerGain,
            RFONE_MIXER_MAX_GAIN,
        ),
        GainType::Vga => legacy(GainType::Vga, VendorRequest::SetVgaGain, RFONE_VGA_MAX_GAIN),
        GainType::LnaAgc => legacy(GainType::LnaAgc, VendorRequest::SetLnaAgc, 1),
        GainType::MixerAgc => legacy(GainType::MixerAgc, VendorRequest::SetMixerAgc, 1),
        GainType::Linearity | GainType::Sensitivity => {
            let index = reverse_gain_table_index(value);
            let (vga, mixer, lna) = if gain_type == GainType::Linearity {
                (
                    RFONE_LINEARITY_VGA_GAINS[index],
                    RFONE_LINEARITY_MIXER_GAINS[index],
                    RFONE_LINEARITY_LNA_GAINS[index],
                )
            } else {
                (
                    RFONE_SENSITIVITY_VGA_GAINS[index],
                    RFONE_SENSITIVITY_MIXER_GAINS[index],
                    RFONE_SENSITIVITY_LNA_GAINS[index],
                )
            };
            (
                vec![
                    VendorControlRequest::legacy_gain(VendorRequest::SetMixerAgc, 0),
                    VendorControlRequest::legacy_gain(VendorRequest::SetLnaAgc, 0),
                    VendorControlRequest::legacy_gain(VendorRequest::SetVgaGain, vga),
                    VendorControlRequest::legacy_gain(VendorRequest::SetMixerGain, mixer),
                    VendorControlRequest::legacy_gain(VendorRequest::SetLnaGain, lna),
                ],
                vec![
                    (GainType::MixerAgc, 0, 1),
                    (GainType::LnaAgc, 0, 1),
                    (GainType::Vga, vga, RFONE_VGA_MAX_GAIN),
                    (GainType::Mixer, mixer, RFONE_MIXER_MAX_GAIN),
                    (GainType::Lna, lna, RFONE_LNA_MAX_GAIN),
                    (gain_type, value.min(21), 21),
                ],
            )
        }
    }
}

fn sample_rate_config(
    rates: &SampleRateTable,
    sample_type: SampleType,
    mode: DecimationMode,
    samplerate: u32,
) -> Result<SelectedSampleRate> {
    if sample_type == SampleType::Raw {
        if let Some(rate) = rates
            .entries
            .iter()
            .find(|rate| rate.rate_hz.checked_mul(2) == Some(samplerate))
        {
            return selected_table_rate(rates.extended, *rate, samplerate, 1);
        }

        return Ok(SelectedSampleRate {
            param: sample_rate_param(&[], sample_type, samplerate)?,
            hardware_rate: samplerate,
            decimation: 1,
            response_len: if rates.extended { 4 } else { 1 },
            expected_encoding: None,
        });
    }

    let (hardware_rate, decimation) =
        sample_rate_hardware_config(rates, mode, samplerate).unwrap_or((samplerate, 1));
    if let Some(rate) = rates
        .entries
        .iter()
        .find(|rate| rate.rate_hz == hardware_rate)
    {
        return selected_table_rate(rates.extended, *rate, hardware_rate, decimation);
    }
    Ok(SelectedSampleRate {
        param: sample_rate_param(&[], sample_type, hardware_rate)?,
        hardware_rate,
        decimation,
        response_len: if rates.extended { 4 } else { 1 },
        expected_encoding: None,
    })
}

fn sample_rate_hardware_config(
    rates: &SampleRateTable,
    mode: DecimationMode,
    samplerate: u32,
) -> Option<(u32, u32)> {
    let direct_rate = rates
        .entries
        .iter()
        .filter(|rate| rate.supported_by_converter())
        .find(|rate| rate.rate_hz == samplerate)
        .map(|rate| rate.rate_hz);
    if mode == DecimationMode::LowBandwidth
        && let Some(rate) = direct_rate
    {
        return Some((rate, 1));
    }

    let mut best = None;
    for hardware_rate in rates
        .entries
        .iter()
        .filter(|rate| rate.supported_by_converter())
        .map(|rate| rate.rate_hz)
    {
        for decimation in DECIMATION_FACTORS_DESC {
            if hardware_rate / decimation == samplerate {
                best = match best {
                    None => Some((hardware_rate, decimation)),
                    Some((best_hw, best_decimation))
                        if mode == DecimationMode::HighDefinition
                            && (hardware_rate > best_hw
                                || (hardware_rate == best_hw && decimation > best_decimation)) =>
                    {
                        Some((hardware_rate, decimation))
                    }
                    Some((best_hw, best_decimation))
                        if mode == DecimationMode::LowBandwidth
                            && (hardware_rate < best_hw
                                || (hardware_rate == best_hw && decimation > best_decimation)) =>
                    {
                        Some((hardware_rate, decimation))
                    }
                    Some(existing) => Some(existing),
                };
            }
        }
    }
    best
}

fn selected_table_rate(
    extended: bool,
    rate: SampleRateInfo,
    hardware_rate: u32,
    decimation: u32,
) -> Result<SelectedSampleRate> {
    if !rate.supported_by_converter() {
        return Err(Error::Unsupported);
    }
    Ok(SelectedSampleRate {
        param: checked_vendor_param(rate.firmware_index)?,
        hardware_rate,
        decimation,
        response_len: if extended { 4 } else { 1 },
        expected_encoding: extended.then_some((rate.adc_bits, rate.data_format)),
    })
}

fn sample_rate_param(
    rates: &[SampleRateInfo],
    sample_type: SampleType,
    hardware_rate: u32,
) -> Result<u16> {
    if let Some(rate) = rates.iter().find(|rate| rate.rate_hz == hardware_rate) {
        if !rate.supported_by_converter() {
            return Err(Error::Unsupported);
        }
        return checked_vendor_param(rate.firmware_index);
    }
    if hardware_rate < MIN_SAMPLERATE_BY_VALUE {
        return Err(Error::invalid_config(
            "sample_rate_hz",
            "cannot be encoded for the firmware",
        ));
    }
    let rate = if sample_type == SampleType::Float32Iq {
        hardware_rate.saturating_mul(2)
    } else {
        hardware_rate
    };
    checked_vendor_param(rate / 1000)
}

fn validate_samplerate_response(selected: &SelectedSampleRate, response: &[u8]) -> Result<()> {
    if response.first().copied() != Some(1) {
        return Err(Error::protocol(
            "set sample rate",
            "firmware rejected the selected sample rate",
        ));
    }
    if selected.response_len == 1 {
        return Ok(());
    }

    let encoding = (response[1], response[2]);
    if selected
        .expected_encoding
        .is_some_and(|expected| expected != encoding)
    {
        return Err(Error::protocol(
            "set sample rate",
            "firmware response does not match extended sample-rate metadata",
        ));
    }
    if encoding != (LEGACY_ADC_BITS, DATA_FORMAT_RAW_ADC) {
        return Err(Error::Unsupported);
    }
    Ok(())
}

fn bandwidth_param(bandwidths: &[u32], bandwidth: u32) -> Result<u16> {
    if let Some(index) = bandwidths.iter().position(|value| *value == bandwidth) {
        return checked_vendor_param(index);
    }
    if bandwidth >= MIN_BANDWIDTH_BY_VALUE {
        return checked_vendor_param(bandwidth / MIN_BANDWIDTH_BY_VALUE);
    }
    if bandwidth < bandwidths.len() as u32 {
        return checked_vendor_param(bandwidth);
    }
    Err(Error::invalid_config(
        "bandwidth_hz",
        "cannot be encoded as a firmware bandwidth index or kHz value",
    ))
}

fn decode_extended_samplerates(data: &[u8], basic_rates: &[u32]) -> Result<SampleRateTable> {
    let (entries, remainder) = data.as_chunks::<8>();
    if !remainder.is_empty() || entries.len() != basic_rates.len() {
        return Err(Error::protocol(
            "decode extended sample rates",
            "response length does not match the basic sample-rate table",
        ));
    }

    let mut decoded = Vec::with_capacity(entries.len());
    for (firmware_index, (entry, basic_rate)) in entries.iter().zip(basic_rates.iter()).enumerate()
    {
        let rate_hz = u32::from_le_bytes(entry[0..4].try_into().expect("four bytes"));
        if rate_hz != *basic_rate {
            return Err(Error::protocol(
                "decode extended sample rates",
                "extended and basic sample-rate tables disagree",
            ));
        }
        decoded.push(SampleRateInfo {
            rate_hz,
            adc_bits: entry[4],
            data_format: entry[5],
            firmware_index,
        });
    }

    Ok(SampleRateTable {
        entries: decoded,
        extended: true,
    })
}

fn is_unsupported_request(error: &Error) -> bool {
    match error {
        Error::Transfer(nusb::transfer::TransferError::Stall) => true,
        Error::Operation { source, .. } => is_unsupported_request(source),
        _ => false,
    }
}

fn decode_c_string(bytes: &[u8]) -> String {
    let len = bytes
        .iter()
        .position(|byte| *byte == 0)
        .unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..len]).into_owned()
}

fn reverse_gain_table_index(value: u8) -> usize {
    21usize.saturating_sub(value.min(21) as usize)
}

fn serial_from_part_id(part_serial: &PartIdSerialNo) -> Option<u64> {
    let serial = ((part_serial.serial_no[2] as u64) << 32) | part_serial.serial_no[3] as u64;
    (serial != 0).then_some(serial)
}

fn checked_vendor_param(value: impl TryInto<u16>) -> Result<u16> {
    value
        .try_into()
        .map_err(|_| Error::protocol("encode vendor request", "parameter exceeds 16 bits"))
}

fn build_virtual_samplerates(hardware_rates: &[SampleRateInfo]) -> Vec<u32> {
    let mut rates = Vec::with_capacity(hardware_rates.len() * DECIMATION_FACTORS_ASC.len());
    for hardware_rate in hardware_rates
        .iter()
        .filter(|rate| rate.supported_by_converter())
        .map(|rate| rate.rate_hz)
    {
        for decimation in DECIMATION_FACTORS_ASC {
            let effective = hardware_rate / decimation;
            if effective >= MIN_SAMPLERATE_BY_VALUE {
                rates.push(effective);
            }
        }
    }
    rates.sort_unstable_by(|a, b| b.cmp(a));
    rates.dedup();
    rates
}

fn build_raw_samplerates(firmware_iq_rates: &[SampleRateInfo]) -> Vec<u32> {
    firmware_iq_rates
        .iter()
        .filter(|rate| rate.supported_by_converter())
        .filter_map(|rate| rate.rate_hz.checked_mul(2))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIRMWARE_IQ_RATES: [u32; 3] = [10_000_000, 5_000_000, 2_500_000];

    #[derive(Clone, Copy, Debug)]
    struct FailingControl(nusb::transfer::TransferError);

    impl ControlBackend for FailingControl {
        fn control_in(
            &self,
            _request: VendorControlRequest,
        ) -> impl MaybeFuture<Output = Result<Vec<u8>>> + use<> {
            ready(Err(Error::from(self.0)))
        }

        fn control_out(
            &self,
            _request: VendorControlRequest,
        ) -> impl MaybeFuture<Output = Result<()>> + use<> {
            ready(Ok(()))
        }
    }

    fn legacy_rates() -> SampleRateTable {
        SampleRateTable::legacy(FIRMWARE_IQ_RATES.to_vec())
    }

    fn legacy_selection(param: u16, hardware_rate: u32) -> SelectedSampleRate {
        SelectedSampleRate {
            param,
            hardware_rate,
            decimation: 1,
            response_len: 1,
            expected_encoding: None,
        }
    }

    #[test]
    fn raw_samplerates_are_reported_in_adc_samples_per_second() {
        assert_eq!(
            build_raw_samplerates(&legacy_rates().entries),
            [20_000_000, 10_000_000, 5_000_000]
        );
    }

    #[test]
    fn raw_samplerates_map_back_to_firmware_iq_rate_indices() {
        assert_eq!(
            sample_rate_config(
                &legacy_rates(),
                SampleType::Raw,
                DecimationMode::LowBandwidth,
                20_000_000,
            )
            .unwrap(),
            legacy_selection(0, 20_000_000)
        );
        assert_eq!(
            sample_rate_config(
                &legacy_rates(),
                SampleType::Raw,
                DecimationMode::LowBandwidth,
                10_000_000,
            )
            .unwrap(),
            legacy_selection(1, 10_000_000)
        );
        assert_eq!(
            sample_rate_config(
                &legacy_rates(),
                SampleType::Raw,
                DecimationMode::LowBandwidth,
                5_000_000,
            )
            .unwrap(),
            legacy_selection(2, 5_000_000)
        );
    }

    #[test]
    fn non_table_raw_samplerates_use_adc_rate_value_encoding() {
        assert_eq!(
            sample_rate_config(
                &legacy_rates(),
                SampleType::Raw,
                DecimationMode::LowBandwidth,
                12_000_000,
            )
            .unwrap(),
            legacy_selection(12_000, 12_000_000)
        );
    }

    #[test]
    fn extended_samplerates_reject_unsupported_adc_encodings() {
        let rates = SampleRateTable {
            entries: vec![SampleRateInfo {
                rate_hz: 10_000_000,
                adc_bits: 8,
                data_format: DATA_FORMAT_RAW_ADC,
                firmware_index: 0,
            }],
            extended: true,
        };

        assert!(matches!(
            sample_rate_config(
                &rates,
                SampleType::Float32Iq,
                DecimationMode::LowBandwidth,
                10_000_000,
            ),
            Err(Error::Unsupported)
        ));
        assert!(build_virtual_samplerates(&rates.entries).is_empty());
    }

    #[test]
    fn extended_samplerate_response_must_match_queried_metadata() {
        let selected = SelectedSampleRate {
            param: 0,
            hardware_rate: 10_000_000,
            decimation: 1,
            response_len: 4,
            expected_encoding: Some((12, DATA_FORMAT_RAW_ADC)),
        };

        assert!(validate_samplerate_response(&selected, &[1, 12, 0, 0]).is_ok());
        assert!(validate_samplerate_response(&selected, &[1, 8, 0, 0]).is_err());
    }

    #[test]
    fn capabilities_only_fall_back_for_an_unsupported_request() {
        assert_eq!(
            HydraSdr::from_control(FailingControl(nusb::transfer::TransferError::Stall))
                .get_capabilities()
                .wait()
                .unwrap(),
            RFONE_HARDCODED_CAPS
        );
        assert!(
            HydraSdr::from_control(FailingControl(nusb::transfer::TransferError::Fault))
                .get_capabilities()
                .wait()
                .is_err()
        );
    }
}
