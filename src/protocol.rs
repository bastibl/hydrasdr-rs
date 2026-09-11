//! RFOne firmware protocol: typed requests, response decoding, and ordered commands.

use crate::{
    config::DecimationPolicy,
    config::{Config, SampleFormat, SampleMode},
    device::DeviceInfo,
    errors::{Error, Result},
    maybe_future::{Either, MaybeFutureExt, OperationFuture, ready},
    rfone::*,
    usb::ControlBackend,
};
use nusb::{
    MaybeFuture,
    transfer::{ControlIn, ControlOut, ControlType, Recipient},
};
use std::{future::IntoFuture, sync::Arc, time::Duration};

const CONTROL_TIMEOUT: Duration = Duration::from_millis(500);
const EXPECTED_FW_PREFIX: &str = "HydraSDR RF";
const DECIMATION_FACTORS_DESC: [u32; 7] = [64, 32, 16, 8, 4, 2, 1];

/// Receiver state encoded by the firmware.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub(crate) enum ReceiverMode {
    Off = 0,
    Rx = 1,
}

/// RFOne USB vendor request numbers.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
#[allow(clippy::enum_variant_names)]
pub(crate) enum VendorRequest {
    ReceiverMode = 1,
    BoardIdRead = 9,
    VersionStringRead = 10,
    BoardPartIdSerialNoRead = 11,
    SetSamplerate = 12,
    SetFreq = 13,
    SetLnaGain = 14,
    SetMixerGain = 15,
    SetVgaGain = 16,
    SetLnaAgc = 17,
    SetMixerAgc = 18,
    SetRfBiasCmd = 20,
    SetPacking = 26,
    SetRfPort = 28,
}

/// HydraSDR board identifier values encoded by the firmware.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub(crate) enum BoardId {
    ProtoHydraSdr = 0,
    HydraSdrRfOneOfficial = 1,
    Invalid = 0xff,
}

/// Raw board ID value that is not recognized by the driver.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct UnknownBoardId(pub(crate) u8);

impl TryFrom<u8> for BoardId {
    type Error = UnknownBoardId;

    fn try_from(value: u8) -> std::result::Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::ProtoHydraSdr),
            1 => Ok(Self::HydraSdrRfOneOfficial),
            0xff => Ok(Self::Invalid),
            other => Err(UnknownBoardId(other)),
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct InRequest {
    pub(crate) request: VendorRequest,
    pub(crate) value: u16,
    pub(crate) index: u16,
    pub(crate) length: u16,
}

impl InRequest {
    fn new(request: VendorRequest, index: u16, length: u16) -> Self {
        Self {
            request,
            value: 0,
            index,
            length,
        }
    }
    pub(crate) fn encode(&self) -> ControlIn {
        ControlIn {
            control_type: ControlType::Vendor,
            recipient: Recipient::Device,
            request: self.request as u8,
            value: self.value,
            index: self.index,
            length: self.length,
        }
    }
    pub(crate) fn set_samplerate(index: u16) -> Self {
        Self::new(VendorRequest::SetSamplerate, index, 1)
    }
    pub(crate) fn stage_gain(request: VendorRequest, value: u8) -> Self {
        Self::new(request, value.into(), 1)
    }
    pub(crate) fn set_packing(enabled: bool) -> Self {
        Self::new(VendorRequest::SetPacking, u16::from(enabled), 1)
    }
    pub(crate) fn set_rf_port(port: crate::RfPort) -> Self {
        Self::new(VendorRequest::SetRfPort, port as u16, 1)
    }
    fn board_id() -> Self {
        Self::new(VendorRequest::BoardIdRead, 0, 1)
    }
    fn firmware() -> Self {
        Self::new(VendorRequest::VersionStringRead, 0, 255)
    }
    fn serial() -> Self {
        Self::new(VendorRequest::BoardPartIdSerialNoRead, 0, 24)
    }

    fn validate(&self, data: &[u8]) -> Result<()> {
        let minimum = if self.request == VendorRequest::VersionStringRead {
            0
        } else {
            usize::from(self.length)
        };
        if data.len() < minimum {
            return Err(Error::protocol(
                "control transfer",
                "response is shorter than requested",
            ));
        }
        match self.request {
            VendorRequest::SetSamplerate => validate_samplerate_response(data),
            VendorRequest::SetRfPort if data.first() != Some(&1) => Err(Error::protocol(
                "set RF port",
                "firmware rejected the requested RF port",
            )),
            _ => Ok(()),
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct OutRequest {
    pub(crate) request: VendorRequest,
    pub(crate) value: u16,
    pub(crate) index: u16,
    pub(crate) data: Vec<u8>,
}

impl OutRequest {
    pub(crate) fn encode(&self) -> ControlOut<'_> {
        ControlOut {
            control_type: ControlType::Vendor,
            recipient: Recipient::Device,
            request: self.request as u8,
            value: self.value,
            index: self.index,
            data: &self.data,
        }
    }
    pub(crate) fn receiver_mode(mode: ReceiverMode) -> Self {
        Self {
            request: VendorRequest::ReceiverMode,
            value: mode as u16,
            index: 0,
            data: Vec::new(),
        }
    }
    pub(crate) fn set_frequency(hz: u64) -> Self {
        Self {
            request: VendorRequest::SetFreq,
            value: 0,
            index: 0,
            data: hz.to_le_bytes().to_vec(),
        }
    }
    pub(crate) fn set_rf_bias(enabled: bool) -> Self {
        Self {
            request: VendorRequest::SetRfBiasCmd,
            value: 0,
            index: u16::from(enabled),
            data: Vec::new(),
        }
    }
}

pub(crate) fn timeout() -> Duration {
    CONTROL_TIMEOUT
}

#[derive(Clone, Debug)]
pub(crate) enum Command {
    In(InRequest),
    Out(OutRequest),
}
impl From<InRequest> for Command {
    fn from(request: InRequest) -> Self {
        Self::In(request)
    }
}
impl From<OutRequest> for Command {
    fn from(request: OutRequest) -> Self {
        Self::Out(request)
    }
}

pub(crate) fn query<C: ControlBackend>(
    control: &C,
    request: InRequest,
) -> impl MaybeFuture<Output = Result<Vec<u8>>> + use<C> {
    control.control_in(request.clone()).map(move |result| {
        let data = result?;
        request.validate(&data)?;
        Ok(data)
    })
}

pub(crate) fn execute<C: ControlBackend>(
    control: &C,
    command: Command,
) -> impl MaybeFuture<Output = Result<()>> + use<C> {
    match command {
        Command::In(request) => Either::left(query(control, request).map_ok(|_| ())),
        Command::Out(request) => Either::right(control.control_out(request)),
    }
}

/// Executes an ordered command list without a runtime in the blocking path.
pub(crate) struct Sequence<C> {
    control: Arc<C>,
    commands: Vec<Command>,
}
impl<C: ControlBackend> Sequence<C> {
    pub(crate) fn new(control: Arc<C>, commands: Vec<Command>) -> Self {
        Self { control, commands }
    }
}
impl<C: ControlBackend + 'static> IntoFuture for Sequence<C> {
    type Output = Result<()>;
    type IntoFuture = OperationFuture<Self::Output>;
    fn into_future(self) -> Self::IntoFuture {
        Box::pin(async move {
            for command in self.commands {
                execute(self.control.as_ref(), command).await?;
            }
            Ok(())
        })
    }
}
impl<C: ControlBackend + 'static> MaybeFuture for Sequence<C> {
    #[cfg(not(target_arch = "wasm32"))]
    fn wait(self) -> Result<()> {
        for command in self.commands {
            execute(self.control.as_ref(), command).wait()?;
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct ReceiveSettings {
    pub(crate) packing: bool,
    pub(crate) decimation: usize,
}

pub(crate) fn configuration<M: SampleMode>(config: &Config<M>) -> (Vec<Command>, ReceiveSettings) {
    let rate = sample_rate_config(
        M::FORMAT,
        config.decimation_policy_internal(),
        config.sample_rate_hz(),
    )
    .expect("Config contains a validated RFOne sample rate");
    let mut commands = vec![
        OutRequest::set_frequency(config.frequency_hz()).into(),
        InRequest::set_samplerate(rate.param).into(),
        InRequest::set_rf_port(config.rf_port()).into(),
    ];
    commands.extend(
        gain_config_plan(config.gain())
            .into_iter()
            .map(Command::from),
    );
    commands.push(InRequest::set_packing(config.packing_internal()).into());
    commands.push(OutRequest::set_rf_bias(config.bias_tee()).into());
    (
        commands,
        ReceiveSettings {
            packing: config.packing_internal(),
            decimation: rate.decimation as usize,
        },
    )
}

pub(crate) fn device_info<C: ControlBackend>(
    control: &C,
) -> impl MaybeFuture<Output = Result<DeviceInfo>> + use<C> {
    let board = query(control, InRequest::board_id());
    let firmware = query(control, InRequest::firmware());
    let serial = query(control, InRequest::serial());
    board
        .map_err(|e| e.at("reading HydraSDR board ID"))
        .and_then(move |bytes| {
            let board = BoardId::try_from(bytes[0]).map_err(|_| {
                Error::protocol("read board ID", "firmware returned an unknown board ID")
            });
            ready(board)
                .and_then(move |_| firmware.map_err(|e| e.at("reading HydraSDR firmware version")))
        })
        .and_then(move |bytes| {
            let firmware_version = decode_c_string(&bytes);
            let validation = if firmware_version.starts_with(EXPECTED_FW_PREFIX) {
                Ok(())
            } else {
                Err(Error::DeviceNotFound)
            };
            ready(validation)
                .and_then(move |_| {
                    serial.map_err(|e| e.at("reading HydraSDR part ID and serial number"))
                })
                .map(move |result| {
                    let serial = decode_serial(&result?)?;
                    Ok(DeviceInfo {
                        board_name: "HydraSDR RFOne",
                        firmware_version,
                        serial,
                        rf_ports: rf_port_infos(),
                    })
                })
        })
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct SelectedSampleRate {
    pub(crate) param: u16,
    pub(crate) decimation: u32,
}
pub(crate) fn gain_config_plan(gain: crate::GainConfig) -> Vec<InRequest> {
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
        crate::GainConfig::Stages { lna, mixer, vga } => stage_gain_plan(lna, mixer, vga),
    }
}

fn stage_gain_plan(lna: crate::StageGain, mixer: crate::StageGain, vga: u8) -> Vec<InRequest> {
    let mut requests = Vec::new();
    let mut push = |request, value| {
        requests.push(InRequest::stage_gain(request, value));
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
) -> Vec<InRequest> {
    let index = reverse_gain_table_index(value);
    vec![
        InRequest::stage_gain(VendorRequest::SetMixerAgc, 0),
        InRequest::stage_gain(VendorRequest::SetLnaAgc, 0),
        InRequest::stage_gain(VendorRequest::SetVgaGain, vga_gains[index]),
        InRequest::stage_gain(VendorRequest::SetMixerGain, mixer_gains[index]),
        InRequest::stage_gain(VendorRequest::SetLnaGain, lna_gains[index]),
    ]
}

pub(crate) fn sample_rate_config(
    format: SampleFormat,
    policy: DecimationPolicy,
    samplerate: u32,
) -> Result<SelectedSampleRate> {
    if format == SampleFormat::RawAdc {
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
    let candidates = RFONE_FIRMWARE_IQ_SAMPLE_RATES
        .into_iter()
        .filter_map(|rate| {
            let decimation = rate.checked_div(samplerate)?;
            (rate.checked_rem(samplerate) == Some(0)
                && DECIMATION_FACTORS_DESC.contains(&decimation))
            .then_some((rate, decimation))
        });
    match policy {
        DecimationPolicy::LowBandwidth => candidates.min_by_key(|(rate, _)| *rate),
        DecimationPolicy::HighDefinition => candidates.max_by_key(|(rate, _)| *rate),
    }
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

fn checked_vendor_param(value: impl TryInto<u16>) -> Result<u16> {
    value
        .try_into()
        .map_err(|_| Error::protocol("encode vendor request", "parameter exceeds 16 bits"))
}

/// The firmware's final two little-endian words carry the high and low serial halves.
fn decode_serial(bytes: &[u8]) -> Result<Option<u64>> {
    let (words, remainder) = bytes.as_chunks::<4>();
    if !remainder.is_empty() {
        return Err(Error::protocol(
            "decode serial number",
            "response length is not a multiple of four bytes",
        ));
    }
    if words.len() < 6 {
        return Err(Error::protocol(
            "decode serial number",
            "response contains fewer than six words",
        ));
    }
    let serial =
        (u64::from(u32::from_le_bytes(words[4])) << 32) | u64::from(u32::from_le_bytes(words[5]));
    Ok((serial != 0).then_some(serial))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn selection(param: u16, decimation: u32) -> SelectedSampleRate {
        SelectedSampleRate { param, decimation }
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
            SampleFormat::F32Iq,
            DecimationPolicy::HighDefinition,
            78_125,
        )
        .unwrap();
        assert_eq!(selected, selection(1, 64));
    }

    #[test]
    fn raw_samplerates_map_back_to_firmware_iq_rate_indices() {
        assert_eq!(
            sample_rate_config(
                SampleFormat::RawAdc,
                DecimationPolicy::LowBandwidth,
                20_000_000,
            )
            .unwrap(),
            selection(0, 1)
        );
        assert_eq!(
            sample_rate_config(
                SampleFormat::RawAdc,
                DecimationPolicy::LowBandwidth,
                10_000_000,
            )
            .unwrap(),
            selection(1, 1)
        );
        assert_eq!(
            sample_rate_config(
                SampleFormat::RawAdc,
                DecimationPolicy::LowBandwidth,
                5_000_000,
            )
            .unwrap(),
            selection(2, 1)
        );
    }

    #[test]
    fn decimation_policy_selects_among_fixed_firmware_rates() {
        assert_eq!(
            sample_rate_config(
                SampleFormat::F32Iq,
                DecimationPolicy::LowBandwidth,
                2_500_000,
            )
            .unwrap(),
            selection(2, 1)
        );
        assert_eq!(
            sample_rate_config(
                SampleFormat::F32Iq,
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
            sample_rate_config(
                SampleFormat::RawAdc,
                DecimationPolicy::LowBandwidth,
                12_000_000,
            ),
            Err(Error::InvalidConfig { .. })
        ));
        assert!(matches!(
            sample_rate_config(
                SampleFormat::F32Iq,
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
    fn stage_gain_config_produces_a_complete_plan() {
        let gain = crate::GainConfig::Stages {
            lna: crate::StageGain::Manual(3),
            mixer: crate::StageGain::Agc,
            vga: 4,
        };

        assert_eq!(
            gain_config_plan(gain),
            [
                InRequest::stage_gain(VendorRequest::SetLnaAgc, 0),
                InRequest::stage_gain(VendorRequest::SetLnaGain, 3),
                InRequest::stage_gain(VendorRequest::SetMixerAgc, 1),
                InRequest::stage_gain(VendorRequest::SetVgaGain, 4),
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
                InRequest::stage_gain(VendorRequest::SetMixerAgc, 0),
                InRequest::stage_gain(VendorRequest::SetLnaAgc, 0),
                InRequest::stage_gain(VendorRequest::SetVgaGain, RFONE_LINEARITY_VGA_GAINS[index],),
                InRequest::stage_gain(
                    VendorRequest::SetMixerGain,
                    RFONE_LINEARITY_MIXER_GAINS[index],
                ),
                InRequest::stage_gain(VendorRequest::SetLnaGain, RFONE_LINEARITY_LNA_GAINS[index],),
            ]
        );
    }
    #[test]
    fn requests_encode_firmware_wire_values_and_little_endian_frequency() {
        let frequency = OutRequest::set_frequency(0x0102_0304_0506_0708);
        let out = frequency.encode();
        assert_eq!((out.request, out.value, out.index), (13, 0, 0));
        assert_eq!(out.control_type, ControlType::Vendor);
        assert_eq!(out.recipient, Recipient::Device);
        assert_eq!(out.data, &[8, 7, 6, 5, 4, 3, 2, 1]);
        let rate = InRequest::set_samplerate(2).encode();
        assert_eq!(
            (rate.request, rate.value, rate.index, rate.length),
            (12, 0, 2, 1)
        );
        assert_eq!(rate.control_type, ControlType::Vendor);
        assert_eq!(rate.recipient, Recipient::Device);
        assert_eq!(
            OutRequest::receiver_mode(ReceiverMode::Rx).encode().value,
            1
        );
        assert_eq!(OutRequest::set_rf_bias(true).encode().index, 1);
        assert_eq!(
            InRequest::stage_gain(VendorRequest::SetMixerGain, 12)
                .encode()
                .index,
            12
        );
        assert_eq!(InRequest::set_packing(true).encode().request, 26);
        assert_eq!(
            InRequest::set_rf_port(crate::RfPort::Rx0).encode().request,
            28
        );
    }

    #[test]
    fn response_validation_preserves_command_specific_acknowledgements() {
        for request in [
            InRequest::set_samplerate(0),
            InRequest::set_rf_port(crate::RfPort::Rx0),
        ] {
            assert!(request.validate(&[1]).is_ok());
            assert!(request.validate(&[0]).is_err());
            assert!(request.validate(&[]).is_err());
        }
        // Packing and gain replies historically require a byte, with no success-bit interpretation.
        for request in [
            InRequest::set_packing(true),
            InRequest::stage_gain(VendorRequest::SetLnaGain, 1),
        ] {
            assert!(request.validate(&[0]).is_ok());
            assert!(request.validate(&[]).is_err());
        }
    }

    #[test]
    fn serial_decoder_uses_firmware_word_order_and_rejects_short_responses() {
        let mut bytes = vec![0; 16];
        bytes.extend_from_slice(&0x1234_5678_u32.to_le_bytes());
        bytes.extend_from_slice(&0x9abc_def0_u32.to_le_bytes());
        assert_eq!(decode_serial(&bytes).unwrap(), Some(0x1234_5678_9abc_def0));
        assert_eq!(decode_serial(&[0; 24]).unwrap(), None);
        assert!(decode_serial(&bytes[..20]).is_err());
        assert!(decode_serial(&bytes[..23]).is_err());
        assert_eq!(
            decode_c_string(b"HydraSDR RF test\0ignored"),
            "HydraSDR RF test"
        );
    }

    #[test]
    fn configuration_preserves_firmware_command_order() {
        let config = Config::default();
        let (commands, receive) = configuration(&config);
        let ids: Vec<_> = commands
            .iter()
            .map(|command| match command {
                Command::In(r) => r.request as u8,
                Command::Out(r) => r.request as u8,
            })
            .collect();
        assert_eq!(ids, [13, 12, 28, 17, 14, 18, 15, 16, 26, 20]);
        assert_eq!(
            receive,
            ReceiveSettings {
                packing: false,
                decimation: 1
            }
        );
    }
}
