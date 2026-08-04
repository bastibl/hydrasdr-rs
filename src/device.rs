use std::sync::Arc;

use nusb::MaybeFuture;

use crate::commands::{ReceiverMode, VendorRequest};
use crate::config::{Config, RfPort, SampleMode};
use crate::discovery;
use crate::errors::{Error, Result};
use crate::maybe_future::{Either, MaybeFutureExt, ready};
use crate::rfone::{
    RFONE_F32_IQ_SAMPLE_RATES, RFONE_FIRMWARE_IQ_SAMPLE_RATES, RFONE_LINEARITY_LNA_GAINS,
    RFONE_LINEARITY_MIXER_GAINS, RFONE_LINEARITY_VGA_GAINS, RFONE_RAW_ADC_SAMPLE_RATES,
    RFONE_RX_ENDPOINT, RFONE_SENSITIVITY_LNA_GAINS, RFONE_SENSITIVITY_MIXER_GAINS,
    RFONE_SENSITIVITY_VGA_GAINS, rf_port_infos,
};
use crate::streaming::{
    AsyncDirectRxStream, AsyncRawRxStream, AsyncStreamingBackend, StreamingState,
};
#[cfg(not(target_arch = "wasm32"))]
use crate::streaming::{DirectRxStream, RawRxStream, StreamingBackend, StreamingStats};
use crate::types::{BoardId, DecimationPolicy, DeviceInfo, PartIdSerialNo, SampleType};
use crate::usb::control::{
    ControlBackend, NusbControl, VendorControlRequest, decode_part_id_serial,
};

const EXPECTED_FW_PREFIX: &str = "HydraSDR RF";
const VERSION_STRING_SIZE: usize = 255;
const MAX_FREQ_HZ: u64 = 10_000_000_000;
const DECIMATION_FACTORS_DESC: [u32; 7] = [64, 32, 16, 8, 4, 2, 1];

/// Direct HydraSDR device handle.
///
/// The default backend is `nusb`; tests can inject fake control/streaming backends with
/// [`HydraSdr::from_control`] to verify C-parity request packing without hardware.
#[derive(Debug)]
pub(crate) struct HydraSdr<C = NusbControl> {
    control: Arc<C>,
    sample_type: SampleType,
    decimation_policy: DecimationPolicy,
    packing_enabled: bool,
    streaming: StreamingState,
}

struct AppliedConfig {
    sample_type: SampleType,
    decimation_policy: DecimationPolicy,
    decimation: u32,
    packing: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct SelectedSampleRate {
    param: u16,
    decimation: u32,
}

impl<C> HydraSdr<C> {
    /// Build a direct device handle from a control backend.
    pub(crate) fn from_control(control: C) -> Self {
        Self {
            control: Arc::new(control),
            sample_type: SampleType::Float32Iq,
            decimation_policy: DecimationPolicy::LowBandwidth,
            packing_enabled: false,
            streaming: StreamingState::new(),
        }
    }

    /// Return the host-side decimation used by converted receive streams.
    pub(crate) fn streaming_decimation_factor(&self) -> usize {
        self.streaming.decimation_factor()
    }

    /// Return whether packed raw transfers are currently configured.
    pub(crate) fn streaming_packing_enabled(&self) -> bool {
        self.streaming.packing_enabled()
    }

    /// Create an independently owned streaming handle over the same USB interface.
    pub(crate) fn stream_handle(&self) -> Self {
        Self {
            control: Arc::clone(&self.control),
            sample_type: self.sample_type,
            decimation_policy: self.decimation_policy,
            packing_enabled: self.packing_enabled,
            streaming: self.streaming.clone(),
        }
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

    pub(crate) fn into_device_info(
        self,
    ) -> impl MaybeFuture<Output = Result<(Self, DeviceInfo)>> + use<C> {
        let fetch = self.fetch_device_info();
        fetch.map(move |result| Ok((self, result?)))
    }

    fn fetch_device_info(&self) -> impl MaybeFuture<Output = Result<DeviceInfo>> + use<C> {
        let board_id = self.board_id_read();
        let firmware = self.version_string_read();
        let part_serial = self.board_partid_serialno_read();
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
            .map(move |result| {
                let (board_id, firmware, part_serial) = result?;
                Ok(build_device_info(board_id, firmware, part_serial))
            })
    }

    pub(crate) fn configure<M: SampleMode>(
        &mut self,
        config: &Config<M>,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C, M> {
        let operation = self.prepare_config(config);
        operation.map(move |result| {
            self.apply_config_state(result?);
            Ok(())
        })
    }

    pub(crate) fn into_configured<M: SampleMode>(
        mut self,
        config: Config<M>,
    ) -> impl MaybeFuture<Output = Result<Self>> + use<C, M> {
        let operation = self.prepare_config(&config);
        operation.map(move |result| {
            self.apply_config_state(result?);
            Ok(self)
        })
    }

    fn prepare_config<M: SampleMode>(
        &self,
        config: &Config<M>,
    ) -> impl MaybeFuture<Output = Result<AppliedConfig>> + use<C, M> {
        let frequency = config.frequency_hz();
        let sample_rate = config.sample_rate_hz();
        let sample_type = M::FORMAT.sample_type();
        let decimation_policy = config.decimation_policy_internal();
        let port = config.rf_port();
        let gain = config.gain();
        let bias_tee = config.bias_tee();
        let packing = config.packing_internal();

        let control = Arc::clone(&self.control);
        let rate_config = sample_rate_config(sample_type, decimation_policy, sample_rate);
        let gain_requests = gain_config_plan(gain);
        let mut gain_requests = gain_requests.into_iter();
        let [gain0, gain1, gain2, gain3, gain4] = [
            gain_requests.next(),
            gain_requests.next(),
            gain_requests.next(),
            gain_requests.next(),
            gain_requests.next(),
        ];
        let set_frequency = Self::control_out_with(
            Arc::clone(&control),
            VendorControlRequest::set_frequency(frequency),
        );

        set_frequency
            .and_then(move |()| ready(rate_config))
            .and_then({
                let control = Arc::clone(&control);
                move |selected| {
                    Self::control_in_exact_with(
                        control,
                        VendorControlRequest::set_samplerate(selected.param),
                        1,
                    )
                    .map(move |result| {
                        validate_samplerate_response(&result?)?;
                        Ok(selected.decimation)
                    })
                }
            })
            .and_then({
                let control = Arc::clone(&control);
                move |state| {
                    Self::control_in_exact_with(control, VendorControlRequest::set_rf_port(port), 1)
                        .map(move |result| {
                            let response = result?;
                            if response.first().copied() != Some(1) {
                                return Err(Error::protocol(
                                    "set RF port",
                                    "firmware rejected the requested RF port",
                                ));
                            }
                            Ok(state)
                        })
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
                    Self::control_in_exact_with(
                        control,
                        VendorControlRequest::set_packing(u8::from(packing)),
                        1,
                    )
                    .map_ok(move |_| state)
                }
            })
            .and_then(move |state| {
                Self::control_out_with(
                    control,
                    VendorControlRequest::set_rf_bias(u8::from(bias_tee)),
                )
                .map_ok(move |_| state)
            })
            .map(move |result| {
                let decimation = result?;
                Ok(AppliedConfig {
                    sample_type,
                    decimation_policy,
                    decimation,
                    packing,
                })
            })
    }

    fn apply_config_state(&mut self, state: AppliedConfig) {
        self.sample_type = state.sample_type;
        self.decimation_policy = state.decimation_policy;
        self.streaming
            .set_decimation(state.decimation as usize)
            .expect("validated HydraSDR decimation factor");
        self.packing_enabled = state.packing;
        self.streaming.set_packing(state.packing);
    }

    /// Set a supported sample rate by its fixed firmware-table index.
    pub(crate) fn set_samplerate(
        &mut self,
        samplerate: u32,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        self.set_samplerate_for_policy(samplerate, self.decimation_policy)
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
        let requests = gain_config_plan(gain);
        self.apply_gain_plan(requests)
    }

    pub(crate) fn set_packing(
        &mut self,
        enabled: bool,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        self.control_in_exact(VendorControlRequest::set_packing(u8::from(enabled)), 1)
            .map(move |result| {
                result?;
                self.packing_enabled = enabled;
                self.streaming.set_packing(enabled);
                Ok(())
            })
    }

    pub(crate) fn set_decimation_policy(
        &mut self,
        sample_rate_hz: u32,
        policy: DecimationPolicy,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        self.set_samplerate_for_policy(sample_rate_hz, policy)
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

    /// Enable or disable RF input bias power directly.
    pub(crate) fn set_rf_bias(
        &self,
        enabled: bool,
    ) -> impl MaybeFuture<Output = Result<()>> + use<C> {
        self.control_out(VendorControlRequest::set_rf_bias(u8::from(enabled)))
    }

    fn set_samplerate_for_policy(
        &mut self,
        samplerate: u32,
        policy: DecimationPolicy,
    ) -> impl MaybeFuture<Output = Result<()>> + use<'_, C> {
        let control = Arc::clone(&self.control);
        let sample_type = self.sample_type;
        let config = sample_rate_config(sample_type, policy, samplerate);
        ready(config)
            .and_then(move |selected| {
                Self::control_in_exact_with(
                    control,
                    VendorControlRequest::set_samplerate(selected.param),
                    1,
                )
                .map(move |result| {
                    validate_samplerate_response(&result?)?;
                    Ok(selected.decimation)
                })
            })
            .map(move |result| {
                let decimation = result?;
                self.streaming.set_decimation(decimation as usize)?;
                self.decimation_policy = policy;
                Ok(())
            })
    }

    fn apply_gain_plan(
        &mut self,
        requests: Vec<VendorControlRequest>,
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
    pub(crate) fn visible_sample_rates(&self) -> Vec<u32> {
        if !self.sample_type_is_iq() {
            return RFONE_RAW_ADC_SAMPLE_RATES.to_vec();
        }
        RFONE_F32_IQ_SAMPLE_RATES.to_vec()
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
        let bulk_in = match self.control.as_ref().bulk_in(RFONE_RX_ENDPOINT) {
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
        let bulk_in = match self.control.as_ref().bulk_in(RFONE_RX_ENDPOINT) {
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

fn build_device_info(
    _board_id: BoardId,
    firmware_version: String,
    part_serial: PartIdSerialNo,
) -> DeviceInfo {
    DeviceInfo {
        serial: serial_from_part_id(&part_serial),
        board_name: "HydraSDR RFOne",
        firmware_version,
        rf_ports: rf_port_infos(),
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

fn gain_config_plan(gain: crate::GainConfig) -> Vec<VendorControlRequest> {
    match gain {
        crate::GainConfig::Preset(crate::GainPreset::Linearity(value)) => preset_gain_plan(
            value,
            &RFONE_LINEARITY_VGA_GAINS,
            &RFONE_LINEARITY_MIXER_GAINS,
            &RFONE_LINEARITY_LNA_GAINS,
        ),
        crate::GainConfig::Preset(crate::GainPreset::Sensitivity(value)) => preset_gain_plan(
            value,
            &RFONE_SENSITIVITY_VGA_GAINS,
            &RFONE_SENSITIVITY_MIXER_GAINS,
            &RFONE_SENSITIVITY_LNA_GAINS,
        ),
        crate::GainConfig::Stages { lna, mixer, vga } => legacy_stage_gain_plan(lna, mixer, vga),
    }
}

fn legacy_stage_gain_plan(
    lna: crate::StageGain,
    mixer: crate::StageGain,
    vga: u8,
) -> Vec<VendorControlRequest> {
    let mut requests = Vec::new();
    let mut push = |request, value| {
        requests.push(VendorControlRequest::legacy_gain(request, value));
    };
    match lna {
        crate::StageGain::Manual(value) => {
            push(VendorRequest::SetLnaAgc, 0);
            push(VendorRequest::SetLnaGain, value);
        }
        crate::StageGain::Agc => push(VendorRequest::SetLnaAgc, 1),
    }
    match mixer {
        crate::StageGain::Manual(value) => {
            push(VendorRequest::SetMixerAgc, 0);
            push(VendorRequest::SetMixerGain, value);
        }
        crate::StageGain::Agc => push(VendorRequest::SetMixerAgc, 1),
    }
    push(VendorRequest::SetVgaGain, vga);
    requests
}

fn preset_gain_plan(
    value: u8,
    vga_gains: &[u8],
    mixer_gains: &[u8],
    lna_gains: &[u8],
) -> Vec<VendorControlRequest> {
    let index = reverse_gain_table_index(value);
    vec![
        VendorControlRequest::legacy_gain(VendorRequest::SetMixerAgc, 0),
        VendorControlRequest::legacy_gain(VendorRequest::SetLnaAgc, 0),
        VendorControlRequest::legacy_gain(VendorRequest::SetVgaGain, vga_gains[index]),
        VendorControlRequest::legacy_gain(VendorRequest::SetMixerGain, mixer_gains[index]),
        VendorControlRequest::legacy_gain(VendorRequest::SetLnaGain, lna_gains[index]),
    ]
}

fn sample_rate_config(
    sample_type: SampleType,
    policy: DecimationPolicy,
    samplerate: u32,
) -> Result<SelectedSampleRate> {
    if sample_type == SampleType::Raw {
        let index = RFONE_RAW_ADC_SAMPLE_RATES
            .iter()
            .position(|rate| *rate == samplerate)
            .ok_or_else(unsupported_sample_rate)?;
        return Ok(SelectedSampleRate {
            param: checked_vendor_param(index)?,
            decimation: 1,
        });
    }

    let (hardware_rate, decimation) =
        sample_rate_hardware_config(policy, samplerate).ok_or_else(unsupported_sample_rate)?;
    let index = RFONE_FIRMWARE_IQ_SAMPLE_RATES
        .iter()
        .position(|rate| *rate == hardware_rate)
        .expect("selected rate comes from the fixed firmware table");
    Ok(SelectedSampleRate {
        param: checked_vendor_param(index)?,
        decimation,
    })
}

fn sample_rate_hardware_config(policy: DecimationPolicy, samplerate: u32) -> Option<(u32, u32)> {
    let direct_rate = RFONE_FIRMWARE_IQ_SAMPLE_RATES
        .iter()
        .find(|rate| **rate == samplerate)
        .copied();
    if policy == DecimationPolicy::LowBandwidth
        && let Some(rate) = direct_rate
    {
        return Some((rate, 1));
    }

    let mut best = None;
    for hardware_rate in RFONE_FIRMWARE_IQ_SAMPLE_RATES {
        for decimation in DECIMATION_FACTORS_DESC {
            if hardware_rate % decimation == 0 && hardware_rate / decimation == samplerate {
                best = match best {
                    None => Some((hardware_rate, decimation)),
                    Some((best_hw, best_decimation))
                        if policy == DecimationPolicy::HighDefinition
                            && (hardware_rate > best_hw
                                || (hardware_rate == best_hw && decimation > best_decimation)) =>
                    {
                        Some((hardware_rate, decimation))
                    }
                    Some((best_hw, best_decimation))
                        if policy == DecimationPolicy::LowBandwidth
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

fn unsupported_sample_rate() -> Error {
    Error::invalid_config(
        "sample_rate_hz",
        "is not supported by RFOne and the host decimator",
    )
}

fn validate_samplerate_response(response: &[u8]) -> Result<()> {
    if response.first().copied() != Some(1) {
        return Err(Error::protocol(
            "set sample rate",
            "firmware rejected the selected sample rate",
        ));
    }
    Ok(())
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

#[cfg(test)]
mod tests {
    use super::*;

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

    fn selection(param: u16, decimation: u32) -> SelectedSampleRate {
        SelectedSampleRate { param, decimation }
    }

    #[test]
    fn visible_samplerates_come_from_the_fixed_tables() {
        let mut device =
            HydraSdr::from_control(FailingControl(nusb::transfer::TransferError::Fault));
        assert_eq!(device.visible_sample_rates(), RFONE_F32_IQ_SAMPLE_RATES);

        device.sample_type = SampleType::Raw;
        assert_eq!(device.visible_sample_rates(), RFONE_RAW_ADC_SAMPLE_RATES);
    }

    #[test]
    fn fixed_iq_table_contains_exact_decimator_outputs() {
        let mut derived = Vec::new();
        for hardware_rate in RFONE_FIRMWARE_IQ_SAMPLE_RATES {
            for decimation in DECIMATION_FACTORS_DESC {
                if hardware_rate % decimation == 0 {
                    derived.push(hardware_rate / decimation);
                }
            }
        }
        derived.sort_unstable_by(|a, b| b.cmp(a));
        derived.dedup();

        assert_eq!(derived, RFONE_F32_IQ_SAMPLE_RATES);
        assert_eq!(
            sample_rate_hardware_config(DecimationPolicy::HighDefinition, 39_062),
            None
        );
        assert_eq!(
            sample_rate_hardware_config(DecimationPolicy::HighDefinition, 78_125),
            Some((5_000_000, 64))
        );
        let selected = sample_rate_config(
            SampleType::Float32Iq,
            DecimationPolicy::HighDefinition,
            78_125,
        )
        .unwrap();
        assert_eq!(selected, selection(1, 64));
    }

    #[test]
    fn raw_samplerates_map_back_to_firmware_iq_rate_indices() {
        assert_eq!(
            sample_rate_config(SampleType::Raw, DecimationPolicy::LowBandwidth, 20_000_000,)
                .unwrap(),
            selection(0, 1)
        );
        assert_eq!(
            sample_rate_config(SampleType::Raw, DecimationPolicy::LowBandwidth, 10_000_000,)
                .unwrap(),
            selection(1, 1)
        );
        assert_eq!(
            sample_rate_config(SampleType::Raw, DecimationPolicy::LowBandwidth, 5_000_000,)
                .unwrap(),
            selection(2, 1)
        );
    }

    #[test]
    fn decimation_policy_selects_among_fixed_firmware_rates() {
        assert_eq!(
            sample_rate_config(
                SampleType::Float32Iq,
                DecimationPolicy::LowBandwidth,
                2_500_000,
            )
            .unwrap(),
            selection(2, 1)
        );
        assert_eq!(
            sample_rate_config(
                SampleType::Float32Iq,
                DecimationPolicy::HighDefinition,
                2_500_000,
            )
            .unwrap(),
            selection(0, 4)
        );
    }

    #[test]
    fn non_table_samplerates_are_rejected() {
        assert!(matches!(
            sample_rate_config(SampleType::Raw, DecimationPolicy::LowBandwidth, 12_000_000,),
            Err(Error::InvalidConfig { .. })
        ));
        assert!(matches!(
            sample_rate_config(
                SampleType::Float32Iq,
                DecimationPolicy::LowBandwidth,
                10_000_500,
            ),
            Err(Error::InvalidConfig { .. })
        ));
    }

    #[test]
    fn samplerate_response_must_accept_the_fixed_index() {
        assert!(validate_samplerate_response(&[1]).is_ok());
        assert!(validate_samplerate_response(&[0]).is_err());
    }

    #[test]
    fn stage_gain_config_produces_a_complete_legacy_plan() {
        let gain = crate::GainConfig::Stages {
            lna: crate::StageGain::Manual(3),
            mixer: crate::StageGain::Agc,
            vga: 4,
        };

        assert_eq!(
            gain_config_plan(gain),
            [
                VendorControlRequest::legacy_gain(VendorRequest::SetLnaAgc, 0),
                VendorControlRequest::legacy_gain(VendorRequest::SetLnaGain, 3),
                VendorControlRequest::legacy_gain(VendorRequest::SetMixerAgc, 1),
                VendorControlRequest::legacy_gain(VendorRequest::SetVgaGain, 4),
            ]
        );
    }

    #[test]
    fn preset_gain_config_uses_the_fixed_rfone_table() {
        let value = 12;
        let index = reverse_gain_table_index(value);

        assert_eq!(
            gain_config_plan(crate::GainConfig::Preset(crate::GainPreset::Linearity(
                value,
            ))),
            [
                VendorControlRequest::legacy_gain(VendorRequest::SetMixerAgc, 0),
                VendorControlRequest::legacy_gain(VendorRequest::SetLnaAgc, 0),
                VendorControlRequest::legacy_gain(
                    VendorRequest::SetVgaGain,
                    RFONE_LINEARITY_VGA_GAINS[index],
                ),
                VendorControlRequest::legacy_gain(
                    VendorRequest::SetMixerGain,
                    RFONE_LINEARITY_MIXER_GAINS[index],
                ),
                VendorControlRequest::legacy_gain(
                    VendorRequest::SetLnaGain,
                    RFONE_LINEARITY_LNA_GAINS[index],
                ),
            ]
        );
    }
}
