#![allow(dead_code)]

use nusb::MaybeFuture;

use crate::commands::{Capability, GainType, ReceiverMode, VendorRequest};
use crate::config::RfPort;
use crate::discovery;
use crate::errors::{Error, Result, StatusCode};
use crate::rfone::{
    RFONE_HARDCODED_CAPS, RFONE_LINEARITY_LNA_GAINS, RFONE_LINEARITY_MIXER_GAINS,
    RFONE_LINEARITY_VGA_GAINS, RFONE_LNA_MAX_GAIN, RFONE_MAX_FREQ_HZ, RFONE_MIN_FREQ_HZ,
    RFONE_MIXER_MAX_GAIN, RFONE_RX_ENDPOINT, RFONE_SENSITIVITY_LNA_GAINS,
    RFONE_SENSITIVITY_MIXER_GAINS, RFONE_SENSITIVITY_VGA_GAINS, RFONE_VGA_MAX_GAIN,
    default_gain_infos, rf_port_infos,
};
use crate::streaming::{
    AsyncRawRxStream, AsyncStreamingBackend, DirectRxStream, RawRxStream, StreamingBackend,
    StreamingState, StreamingStats, Transfer,
};
use crate::types::{
    BoardId, DecimationMode, DeviceInfo, GainInfo, PartIdSerialNo, SampleType, Temperature,
};
use crate::usb::control::{
    AsyncControlBackend, ControlBackend, NusbControl, VendorControlRequest, decode_part_id_serial,
    decode_u32_le_words,
};

const EXPECTED_FW_PREFIX: &str = "HydraSDR RF";
const VERSION_STRING_SIZE: usize = 255;
const MIN_SAMPLERATE_BY_VALUE: u32 = 10_000;
const MIN_BANDWIDTH_BY_VALUE: u32 = 1_000;
const MAX_FREQ_HZ: u64 = 10_000_000_000;
const DECIMATION_FACTORS_ASC: [u32; 7] = [1, 2, 4, 8, 16, 32, 64];
const DECIMATION_FACTORS_DESC: [u32; 7] = [64, 32, 16, 8, 4, 2, 1];

/// Direct HydraSDR device handle.
///
/// The default backend is `nusb`; tests can inject fake control/streaming backends with
/// [`HydraSdr::from_control`] to verify C-parity request packing without hardware.
#[derive(Debug)]
pub struct HydraSdr<C = NusbControl> {
    control: C,
    sample_type: SampleType,
    sample_rates: Vec<u32>,
    bandwidths: Vec<u32>,
    features: Option<u32>,
    gains: Vec<GainInfo>,
    current_samplerate: u32,
    hardware_samplerate: u32,
    decimation_factor: u32,
    decimation_mode: DecimationMode,
    current_bandwidth: u32,
    packing_enabled: bool,
    reset_command: bool,
    streaming: StreamingState,
}

impl<C: ControlBackend> HydraSdr<C> {
    /// Build a direct device handle from a control backend.
    pub fn from_control(control: C) -> Self {
        Self {
            control,
            sample_type: SampleType::Float32Iq,
            sample_rates: Vec::new(),
            bandwidths: Vec::new(),
            features: None,
            gains: default_gain_infos(),
            current_samplerate: 0,
            hardware_samplerate: 0,
            decimation_factor: 1,
            decimation_mode: DecimationMode::LowBandwidth,
            current_bandwidth: 0,
            packing_enabled: false,
            reset_command: false,
            streaming: StreamingState::new(),
        }
    }

    /// Borrow the underlying control backend.
    pub fn control(&self) -> &C {
        &self.control
    }

    /// Close the direct handle.
    ///
    /// This consumes `self`, matching the explicit C close step while relying on Rust drops for
    /// the USB resources owned by the backend.
    pub fn close(self) -> Result<()> {
        Ok(())
    }

    /// Read the board ID, matching `hydrasdr_board_id_read`.
    pub fn board_id_read(&self) -> Result<BoardId> {
        let data = self.control_in_exact(VendorControlRequest::board_id_read(), 1)?;
        BoardId::try_from(data[0]).map_err(|_| Error::Status(StatusCode::Other))
    }

    /// Read the firmware version C string, matching `hydrasdr_version_string_read`.
    pub fn version_string_read(&self) -> Result<String> {
        let data = self.control_in_min(
            VendorControlRequest::version_string_read(VERSION_STRING_SIZE),
            0,
        )?;
        Ok(decode_c_string(&data))
    }

    /// Read part ID and serial-number words, matching `hydrasdr_board_partid_serialno_read`.
    pub fn board_partid_serialno_read(&self) -> Result<PartIdSerialNo> {
        let data = self.control_in_exact(VendorControlRequest::board_partid_serialno_read(), 24)?;
        decode_part_id_serial(&data)
    }

    /// Read the primary firmware capability word.
    ///
    /// If firmware does not support the request, the RFOne hard-coded C capability mask is used.
    pub fn get_capabilities(&self) -> Result<u32> {
        match self.control_in_exact(VendorControlRequest::get_capabilities(0), 4) {
            Ok(data) => Ok(u32::from_le_bytes(
                data[0..4].try_into().expect("four bytes"),
            )),
            Err(_) => Ok(RFONE_HARDCODED_CAPS),
        }
    }

    /// Read reserved capability words when firmware provides them.
    pub fn get_capabilities_reserved(&self) -> Result<[u32; 3]> {
        let mut reserved = [0; 3];
        for (i, slot) in reserved.iter_mut().enumerate() {
            if let Ok(data) =
                self.control_in_exact(VendorControlRequest::get_capabilities(i as u16 + 1), 4)
            {
                *slot = u32::from_le_bytes(data[0..4].try_into().expect("four bytes"));
            }
        }
        Ok(reserved)
    }

    /// Build direct device metadata from firmware queries and RFOne static tables.
    pub fn get_device_info(&mut self) -> Result<DeviceInfo> {
        let board_id = self.board_id_read()?;
        let firmware_version = self.version_string_read()?;
        let part_serial = self.board_partid_serialno_read()?;
        let features = self.get_capabilities()?;
        self.features = Some(features);
        let features_reserved = self.get_capabilities_reserved()?;
        Ok(self.build_device_info(
            board_id,
            firmware_version,
            part_serial,
            features,
            features_reserved,
        ))
    }

    /// Read supported sample rates with the C count-then-list protocol.
    ///
    /// IQ sample modes return the C-style virtual rate table built from hardware rates and
    /// supported DDC decimation factors.
    pub fn get_samplerates(&mut self) -> Result<Vec<u32>> {
        let count = self.read_count(VendorControlRequest::get_samplerates_count(false))?;
        let rates =
            self.read_u32_list(VendorControlRequest::get_samplerates(count, false), count)?;
        self.sample_rates = rates;
        Ok(self.visible_sample_rates())
    }

    /// Set sample rate by C-compatible index or kHz fallback calculation.
    pub fn set_samplerate(&mut self, samplerate: u32) -> Result<()> {
        let (rate_param, hardware_samplerate, decimation_factor) =
            self.sample_rate_config(samplerate)?;
        self.control_in_min(VendorControlRequest::set_samplerate(rate_param, 1), 1)?;
        self.streaming.set_decimation(decimation_factor as usize)?;
        self.current_samplerate = samplerate;
        self.hardware_samplerate = hardware_samplerate;
        self.decimation_factor = decimation_factor;
        Ok(())
    }

    /// Read supported bandwidths with the C count-then-list protocol.
    pub fn get_bandwidths(&mut self) -> Result<Vec<u32>> {
        let count = self.read_count(VendorControlRequest::get_bandwidths_count())?;
        let bandwidths = self.read_u32_list(VendorControlRequest::get_bandwidths(count), count)?;
        self.bandwidths = bandwidths.clone();
        Ok(bandwidths)
    }

    /// Set analog bandwidth by C-compatible index or kHz fallback calculation.
    pub fn set_bandwidth(&mut self, bandwidth: u32) -> Result<()> {
        let bandwidth_param = self.bandwidth_param(bandwidth)?;
        self.control_in_min(VendorControlRequest::set_bandwidth(bandwidth_param), 1)?;
        self.current_bandwidth = bandwidth;
        Ok(())
    }

    /// Set tuning frequency in Hz, matching `hydrasdr_set_freq` validation.
    pub fn set_freq(&mut self, freq_hz: u64) -> Result<()> {
        if freq_hz == 0 || freq_hz > MAX_FREQ_HZ {
            return Err(Error::Status(StatusCode::InvalidParam));
        }
        self.control_out(VendorControlRequest::set_frequency(freq_hz))
    }

    /// Set legacy LNA gain; values above the RFOne maximum are clamped like the C driver.
    pub fn set_lna_gain(&mut self, value: u8) -> Result<()> {
        self.set_legacy_gain(
            GainType::Lna,
            VendorRequest::SetLnaGain,
            value,
            RFONE_LNA_MAX_GAIN,
        )
    }

    /// Set legacy mixer gain; values above the RFOne maximum are clamped like the C driver.
    pub fn set_mixer_gain(&mut self, value: u8) -> Result<()> {
        self.set_legacy_gain(
            GainType::Mixer,
            VendorRequest::SetMixerGain,
            value,
            RFONE_MIXER_MAX_GAIN,
        )
    }

    /// Set legacy VGA gain; values above the RFOne maximum are clamped like the C driver.
    pub fn set_vga_gain(&mut self, value: u8) -> Result<()> {
        self.set_legacy_gain(
            GainType::Vga,
            VendorRequest::SetVgaGain,
            value,
            RFONE_VGA_MAX_GAIN,
        )
    }

    /// Enable or disable LNA AGC through the legacy request.
    pub fn set_lna_agc(&mut self, value: u8) -> Result<()> {
        self.set_legacy_gain(GainType::LnaAgc, VendorRequest::SetLnaAgc, value, 1)
    }

    /// Enable or disable mixer AGC through the legacy request.
    pub fn set_mixer_agc(&mut self, value: u8) -> Result<()> {
        self.set_legacy_gain(GainType::MixerAgc, VendorRequest::SetMixerAgc, value, 1)
    }

    /// Set a gain through the extended gain API when available, otherwise through C fallbacks.
    pub fn set_gain(&mut self, gain_type: GainType, value: u8) -> Result<()> {
        if self.features.unwrap_or(0) & Capability::ExtendedGain.bits() != 0 {
            self.control_in_min(VendorControlRequest::unified_gain(gain_type, value), 1)?;
            self.update_gain_cache(gain_type, value, value.max(1));
            return Ok(());
        }
        match gain_type {
            GainType::Lna => self.set_lna_gain(value),
            GainType::Mixer => self.set_mixer_gain(value),
            GainType::Vga => self.set_vga_gain(value),
            GainType::LnaAgc => self.set_lna_agc(value),
            GainType::MixerAgc => self.set_mixer_agc(value),
            GainType::Linearity => self.set_linearity_gain(value),
            GainType::Sensitivity => self.set_sensitivity_gain(value),
            GainType::Rf | GainType::Filter | GainType::RfAgc | GainType::FilterAgc => {
                Err(Error::Status(StatusCode::Unsupported))
            }
            GainType::Count => Err(Error::Status(StatusCode::InvalidParam)),
        }
    }

    /// Return the cached descriptor for one gain type.
    pub fn get_gain(&self, gain_type: GainType) -> Result<GainInfo> {
        self.gains
            .iter()
            .copied()
            .find(|gain| gain.gain_type == gain_type)
            .ok_or(Error::Status(StatusCode::Unsupported))
    }

    /// Return all cached gain descriptors.
    pub fn get_all_gains(&self) -> Result<Vec<GainInfo>> {
        Ok(self.gains.clone())
    }

    /// Apply the RFOne linearity preset table, matching the C gain choreography.
    pub fn set_linearity_gain(&mut self, value: u8) -> Result<()> {
        let index = reverse_gain_table_index(value);
        self.set_mixer_agc(0)?;
        self.set_lna_agc(0)?;
        self.set_vga_gain(RFONE_LINEARITY_VGA_GAINS[index])?;
        self.set_mixer_gain(RFONE_LINEARITY_MIXER_GAINS[index])?;
        self.set_lna_gain(RFONE_LINEARITY_LNA_GAINS[index])?;
        self.update_gain_cache(GainType::Linearity, value.min(21), 21);
        Ok(())
    }

    /// Apply the RFOne sensitivity preset table, matching the C gain choreography.
    pub fn set_sensitivity_gain(&mut self, value: u8) -> Result<()> {
        let index = reverse_gain_table_index(value);
        self.set_mixer_agc(0)?;
        self.set_lna_agc(0)?;
        self.set_vga_gain(RFONE_SENSITIVITY_VGA_GAINS[index])?;
        self.set_mixer_gain(RFONE_SENSITIVITY_MIXER_GAINS[index])?;
        self.set_lna_gain(RFONE_SENSITIVITY_LNA_GAINS[index])?;
        self.update_gain_cache(GainType::Sensitivity, value.min(21), 21);
        Ok(())
    }

    /// Control RF bias tee power through the C vendor request.
    pub fn set_rf_bias(&mut self, value: u8) -> Result<()> {
        self.control_out(VendorControlRequest::set_rf_bias(value))
    }

    /// Enable or disable C packed-sample mode before streaming.
    pub fn set_packing(&mut self, value: u8) -> Result<()> {
        self.control_in_min(VendorControlRequest::set_packing(value), 1)?;
        self.packing_enabled = value == 1;
        self.streaming.set_packing(self.packing_enabled)?;
        Ok(())
    }

    /// Select an RF input port and require the firmware success byte used by the C API.
    pub fn set_rf_port(&mut self, port: RfPort) -> Result<()> {
        let response = self.control_in_min(VendorControlRequest::set_rf_port(port), 1)?;
        if response.first().copied() != Some(1) {
            return Err(Error::Status(StatusCode::InvalidParam));
        }
        Ok(())
    }

    /// Send the reset command and remember that stop paths should not force receiver off again.
    pub fn reset(&mut self) -> Result<()> {
        let _ = self.control.control_in(VendorControlRequest::reset());
        self.reset_command = true;
        Ok(())
    }

    /// Set the sample type tracked by the direct streaming path.
    pub fn set_sample_type(&mut self, sample_type: SampleType) -> Result<()> {
        if sample_type == SampleType::End {
            return Err(Error::Status(StatusCode::InvalidParam));
        }
        self.sample_type = sample_type;
        Ok(())
    }

    /// Return the currently selected sample type.
    pub fn get_sample_type(&self) -> SampleType {
        self.sample_type
    }

    /// Select how virtual IQ sample rates choose a hardware rate and host-side DDC decimation.
    ///
    /// `LowBandwidth` prefers a direct firmware hardware rate when one is available. `HighDefinition`
    /// prefers the highest compatible hardware rate and decimates in the host converter.
    pub fn set_decimation_mode(&mut self, mode: DecimationMode) -> Result<()> {
        if self.decimation_mode == mode {
            return Ok(());
        }

        let previous = self.decimation_mode;
        self.decimation_mode = mode;
        if self.current_samplerate != 0
            && let Err(err) = self.set_samplerate(self.current_samplerate)
        {
            self.decimation_mode = previous;
            return Err(err);
        }
        Ok(())
    }

    /// Return the currently selected virtual-rate decimation mode.
    pub fn decimation_mode(&self) -> DecimationMode {
        self.decimation_mode
    }

    /// Write a GPIO value using the C port/pin packing.
    pub fn gpio_write(&self, port: u8, pin: u8, value: u8) -> Result<()> {
        self.control_out(VendorControlRequest::gpio_write(port, pin, value)?)
    }

    /// Read a GPIO value using the C port/pin packing.
    pub fn gpio_read(&self, port: u8, pin: u8) -> Result<u8> {
        let data = self.control_in_exact(VendorControlRequest::gpio_read(port, pin)?, 1)?;
        Ok(data[0])
    }

    /// Write a GPIO direction bit using the C port/pin packing.
    pub fn gpiodir_write(&self, port: u8, pin: u8, value: u8) -> Result<()> {
        self.control_out(VendorControlRequest::gpiodir_write(port, pin, value)?)
    }

    /// Read a GPIO direction bit using the C port/pin packing.
    pub fn gpiodir_read(&self, port: u8, pin: u8) -> Result<u8> {
        let data = self.control_in_exact(VendorControlRequest::gpiodir_read(port, pin)?, 1)?;
        Ok(data[0])
    }

    /// Write one clock-generator register.
    pub fn clockgen_write(&self, reg: u8, value: u8) -> Result<()> {
        self.control_out(VendorControlRequest::clockgen_write(reg, value))
    }

    /// Read one clock-generator register.
    pub fn clockgen_read(&self, reg: u8) -> Result<u8> {
        let data = self.control_in_exact(VendorControlRequest::clockgen_read(reg), 1)?;
        Ok(data[0])
    }

    /// Write one RF frontend register through the direct vendor request.
    pub fn rf_frontend_write(&self, reg: u16, value: u32) -> Result<()> {
        self.control_out(VendorControlRequest::rf_frontend_write(reg, value))
    }

    /// Read one RF frontend register through the direct vendor request.
    pub fn rf_frontend_read(&self, reg: u16) -> Result<u32> {
        let data = self.control_in_exact(VendorControlRequest::rf_frontend_read(reg), 1)?;
        Ok(data[0] as u32)
    }

    /// Erase the whole SPI flash; this is a direct C-parity hazardous hardware operation.
    pub fn spiflash_erase(&self) -> Result<()> {
        self.control_out(VendorControlRequest::spiflash_erase())
    }

    /// Erase one SPI flash sector.
    pub fn spiflash_erase_sector(&self, sector: u16) -> Result<()> {
        self.control_out(VendorControlRequest::spiflash_erase_sector(sector))
    }

    /// Write bytes to SPI flash using the C address packing.
    pub fn spiflash_write(&self, addr: u32, data: &[u8]) -> Result<()> {
        self.control_out(VendorControlRequest::spiflash_write(addr, data)?)
    }

    /// Read bytes from SPI flash using the C address packing.
    pub fn spiflash_read(&self, addr: u32, len: u16) -> Result<Vec<u8>> {
        self.control_in_exact(
            VendorControlRequest::spiflash_read(addr, len)?,
            len as usize,
        )
    }

    /// Temperature query placeholder; RFOne currently maps this to unsupported.
    pub fn get_temperature(&self) -> Result<Temperature> {
        Err(Error::Status(StatusCode::Unsupported))
    }

    /// Set receiver mode directly.
    pub fn receiver_mode(&self, mode: ReceiverMode) -> Result<()> {
        self.control_out(VendorControlRequest::receiver_mode(mode))
    }

    /// Request RX stop and send receiver-off unless a reset command already owns shutdown.
    pub fn stop_rx(&mut self) -> Result<()> {
        self.streaming.request_stop();
        self.receiver_off_if_needed()
    }

    /// Send receiver-off unless a reset command already owns shutdown.
    pub(crate) fn receiver_off_if_needed(&self) -> Result<()> {
        if !self.reset_command {
            self.receiver_mode(ReceiverMode::Off)?;
        }
        Ok(())
    }

    /// Report whether the direct streaming state machine is currently active.
    pub fn is_streaming(&self) -> bool {
        self.streaming.is_streaming()
    }

    /// Return counters collected by the direct streaming state machine.
    pub fn streaming_stats(&self) -> StreamingStats {
        self.streaming.stats()
    }

    fn sample_rate_config(&mut self, samplerate: u32) -> Result<(u32, u32, u32)> {
        if self.sample_rates.is_empty() {
            let _ = self.get_samplerates();
        }
        let (hardware_samplerate, decimation_factor) = self
            .sample_rate_hardware_config(samplerate)
            .unwrap_or((samplerate, 1));
        let rate_param = self.sample_rate_param_for_hardware_rate(hardware_samplerate)?;
        Ok((rate_param, hardware_samplerate, decimation_factor))
    }

    fn bandwidth_param(&mut self, bandwidth: u32) -> Result<u32> {
        if self.bandwidths.is_empty() {
            let _ = self.get_bandwidths();
        }
        if let Some(index) = self.bandwidths.iter().position(|value| *value == bandwidth) {
            return Ok(index as u32);
        }
        if bandwidth >= MIN_BANDWIDTH_BY_VALUE {
            return Ok(bandwidth / MIN_BANDWIDTH_BY_VALUE);
        }
        if bandwidth < self.bandwidths.len() as u32 {
            return Ok(bandwidth);
        }
        Err(Error::Status(StatusCode::InvalidParam))
    }

    fn set_legacy_gain(
        &mut self,
        gain_type: GainType,
        request: VendorRequest,
        value: u8,
        max_value: u8,
    ) -> Result<()> {
        let value = value.min(max_value);
        self.control_in_min(VendorControlRequest::legacy_gain(request, value), 1)?;
        self.update_gain_cache(gain_type, value, max_value);
        Ok(())
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

    fn read_count(&self, request: VendorControlRequest) -> Result<u32> {
        let data = self.control_in_exact(request, 4)?;
        Ok(u32::from_le_bytes(
            data[0..4].try_into().expect("four bytes"),
        ))
    }

    fn read_u32_list(&self, request: VendorControlRequest, count: u32) -> Result<Vec<u32>> {
        let data = self.control_in_exact(request, count as usize * 4)?;
        decode_u32_le_words(&data)
    }

    fn control_in_exact(&self, request: VendorControlRequest, len: usize) -> Result<Vec<u8>> {
        let data = self.control.control_in(request)?;
        if data.len() < len {
            return Err(Error::Status(StatusCode::LibUsb));
        }
        Ok(data)
    }

    fn control_in_min(&self, request: VendorControlRequest, len: usize) -> Result<Vec<u8>> {
        self.control_in_exact(request, len)
    }

    fn control_out(&self, request: VendorControlRequest) -> Result<()> {
        self.control.control_out(request)
    }
}

impl<C> HydraSdr<C> {
    pub(crate) fn update_cached_device_info(&self, info: &mut DeviceInfo) {
        let _ = self;
        let _ = info;
    }

    pub(crate) const fn decimation_factor(&self) -> usize {
        self.decimation_factor as usize
    }

    fn build_device_info(
        &self,
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

    fn visible_sample_rates(&self) -> Vec<u32> {
        if !self.sample_type_is_iq() {
            return self.sample_rates.clone();
        }
        build_virtual_samplerates(&self.sample_rates)
    }

    fn sample_type_is_iq(&self) -> bool {
        matches!(
            self.sample_type,
            SampleType::Float32Iq | SampleType::Int16Iq | SampleType::Int8Iq | SampleType::Uint8Iq
        )
    }

    fn sample_rate_hardware_config(&self, samplerate: u32) -> Option<(u32, u32)> {
        let direct_rate = self
            .sample_rates
            .iter()
            .find(|rate| **rate == samplerate)
            .copied();

        if !self.sample_type_is_iq() {
            return direct_rate.map(|rate| (rate, 1));
        }

        if self.decimation_mode == DecimationMode::LowBandwidth
            && let Some(rate) = direct_rate
        {
            return Some((rate, 1));
        }

        let mut best = None;
        for hardware_rate in &self.sample_rates {
            for decimation in DECIMATION_FACTORS_DESC {
                if hardware_rate / decimation == samplerate {
                    best = match best {
                        None => Some((*hardware_rate, decimation)),
                        Some((best_hw, best_decimation))
                            if self.decimation_mode == DecimationMode::HighDefinition
                                && (*hardware_rate > best_hw
                                    || (*hardware_rate == best_hw
                                        && decimation > best_decimation)) =>
                        {
                            Some((*hardware_rate, decimation))
                        }
                        Some((best_hw, best_decimation))
                            if self.decimation_mode == DecimationMode::LowBandwidth
                                && (*hardware_rate < best_hw
                                    || (*hardware_rate == best_hw
                                        && decimation > best_decimation)) =>
                        {
                            Some((*hardware_rate, decimation))
                        }
                        Some(existing) => Some(existing),
                    };
                }
            }
        }
        best
    }

    fn sample_rate_param_for_hardware_rate(&self, hardware_samplerate: u32) -> Result<u32> {
        if let Some(index) = self
            .sample_rates
            .iter()
            .position(|rate| *rate == hardware_samplerate)
        {
            return Ok(index as u32);
        }
        if hardware_samplerate < MIN_SAMPLERATE_BY_VALUE {
            return Err(Error::Status(StatusCode::InvalidParam));
        }
        let mut rate_param = hardware_samplerate;
        if self.sample_type_is_iq() {
            rate_param = rate_param.saturating_mul(2);
        }
        Ok(rate_param / 1000)
    }
}

impl<C: AsyncControlBackend> HydraSdr<C> {
    /// Async counterpart to [`HydraSdr::board_id_read`].
    pub async fn board_id_read_async(&self) -> Result<BoardId> {
        let data = self
            .control_in_exact_async(VendorControlRequest::board_id_read(), 1)
            .await?;
        BoardId::try_from(data[0]).map_err(|_| Error::Status(StatusCode::Other))
    }

    /// Async counterpart to [`HydraSdr::version_string_read`].
    pub async fn version_string_read_async(&self) -> Result<String> {
        let data = self
            .control_in_min_async(
                VendorControlRequest::version_string_read(VERSION_STRING_SIZE),
                0,
            )
            .await?;
        Ok(decode_c_string(&data))
    }

    /// Async counterpart to [`HydraSdr::board_partid_serialno_read`].
    pub async fn board_partid_serialno_read_async(&self) -> Result<PartIdSerialNo> {
        let data = self
            .control_in_exact_async(VendorControlRequest::board_partid_serialno_read(), 24)
            .await?;
        decode_part_id_serial(&data)
    }

    /// Async counterpart to [`HydraSdr::get_capabilities`].
    pub async fn get_capabilities_async(&self) -> Result<u32> {
        match self
            .control_in_exact_async(VendorControlRequest::get_capabilities(0), 4)
            .await
        {
            Ok(data) => Ok(u32::from_le_bytes(
                data[0..4].try_into().expect("four bytes"),
            )),
            Err(_) => Ok(RFONE_HARDCODED_CAPS),
        }
    }

    /// Async counterpart to [`HydraSdr::get_capabilities_reserved`].
    pub async fn get_capabilities_reserved_async(&self) -> Result<[u32; 3]> {
        let mut reserved = [0; 3];
        for (i, slot) in reserved.iter_mut().enumerate() {
            if let Ok(data) = self
                .control_in_exact_async(VendorControlRequest::get_capabilities(i as u16 + 1), 4)
                .await
            {
                *slot = u32::from_le_bytes(data[0..4].try_into().expect("four bytes"));
            }
        }
        Ok(reserved)
    }

    /// Async counterpart to [`HydraSdr::get_device_info`].
    pub async fn get_device_info_async(&mut self) -> Result<DeviceInfo> {
        let board_id = self.board_id_read_async().await?;
        let firmware_version = self.version_string_read_async().await?;
        let part_serial = self.board_partid_serialno_read_async().await?;
        let features = self.get_capabilities_async().await?;
        self.features = Some(features);
        let features_reserved = self.get_capabilities_reserved_async().await?;
        Ok(self.build_device_info(
            board_id,
            firmware_version,
            part_serial,
            features,
            features_reserved,
        ))
    }

    /// Async counterpart to [`HydraSdr::get_samplerates`].
    pub async fn get_samplerates_async(&mut self) -> Result<Vec<u32>> {
        let count = self
            .read_count_async(VendorControlRequest::get_samplerates_count(false))
            .await?;
        let rates = self
            .read_u32_list_async(VendorControlRequest::get_samplerates(count, false), count)
            .await?;
        self.sample_rates = rates;
        Ok(self.visible_sample_rates())
    }

    /// Async counterpart to [`HydraSdr::set_samplerate`].
    pub async fn set_samplerate_async(&mut self, samplerate: u32) -> Result<()> {
        let (rate_param, hardware_samplerate, decimation_factor) =
            self.sample_rate_config_async(samplerate).await?;
        self.control_in_min_async(VendorControlRequest::set_samplerate(rate_param, 1), 1)
            .await?;
        self.streaming.set_decimation(decimation_factor as usize)?;
        self.current_samplerate = samplerate;
        self.hardware_samplerate = hardware_samplerate;
        self.decimation_factor = decimation_factor;
        Ok(())
    }

    /// Async counterpart to [`HydraSdr::set_decimation_mode`].
    pub async fn set_decimation_mode_async(&mut self, mode: DecimationMode) -> Result<()> {
        if self.decimation_mode == mode {
            return Ok(());
        }

        let previous = self.decimation_mode;
        self.decimation_mode = mode;
        if self.current_samplerate != 0
            && let Err(err) = self.set_samplerate_async(self.current_samplerate).await
        {
            self.decimation_mode = previous;
            return Err(err);
        }
        Ok(())
    }

    /// Async counterpart to [`HydraSdr::get_bandwidths`].
    pub async fn get_bandwidths_async(&mut self) -> Result<Vec<u32>> {
        let count = self
            .read_count_async(VendorControlRequest::get_bandwidths_count())
            .await?;
        let bandwidths = self
            .read_u32_list_async(VendorControlRequest::get_bandwidths(count), count)
            .await?;
        self.bandwidths = bandwidths.clone();
        Ok(bandwidths)
    }

    /// Async counterpart to [`HydraSdr::set_bandwidth`].
    pub async fn set_bandwidth_async(&mut self, bandwidth: u32) -> Result<()> {
        let bandwidth_param = self.bandwidth_param_async(bandwidth).await?;
        self.control_in_min_async(VendorControlRequest::set_bandwidth(bandwidth_param), 1)
            .await?;
        self.current_bandwidth = bandwidth;
        Ok(())
    }

    /// Async counterpart to [`HydraSdr::set_freq`].
    pub async fn set_freq_async(&mut self, freq_hz: u64) -> Result<()> {
        if freq_hz == 0 || freq_hz > MAX_FREQ_HZ {
            return Err(Error::Status(StatusCode::InvalidParam));
        }
        self.control_out_async(VendorControlRequest::set_frequency(freq_hz))
            .await
    }

    /// Async counterpart to [`HydraSdr::set_lna_gain`].
    pub async fn set_lna_gain_async(&mut self, value: u8) -> Result<()> {
        self.set_legacy_gain_async(
            GainType::Lna,
            VendorRequest::SetLnaGain,
            value,
            RFONE_LNA_MAX_GAIN,
        )
        .await
    }

    /// Async counterpart to [`HydraSdr::set_mixer_gain`].
    pub async fn set_mixer_gain_async(&mut self, value: u8) -> Result<()> {
        self.set_legacy_gain_async(
            GainType::Mixer,
            VendorRequest::SetMixerGain,
            value,
            RFONE_MIXER_MAX_GAIN,
        )
        .await
    }

    /// Async counterpart to [`HydraSdr::set_vga_gain`].
    pub async fn set_vga_gain_async(&mut self, value: u8) -> Result<()> {
        self.set_legacy_gain_async(
            GainType::Vga,
            VendorRequest::SetVgaGain,
            value,
            RFONE_VGA_MAX_GAIN,
        )
        .await
    }

    /// Async counterpart to [`HydraSdr::set_lna_agc`].
    pub async fn set_lna_agc_async(&mut self, value: u8) -> Result<()> {
        self.set_legacy_gain_async(GainType::LnaAgc, VendorRequest::SetLnaAgc, value, 1)
            .await
    }

    /// Async counterpart to [`HydraSdr::set_mixer_agc`].
    pub async fn set_mixer_agc_async(&mut self, value: u8) -> Result<()> {
        self.set_legacy_gain_async(GainType::MixerAgc, VendorRequest::SetMixerAgc, value, 1)
            .await
    }

    /// Async counterpart to [`HydraSdr::set_gain`].
    pub async fn set_gain_async(&mut self, gain_type: GainType, value: u8) -> Result<()> {
        if self.features.unwrap_or(0) & Capability::ExtendedGain.bits() != 0 {
            self.control_in_min_async(VendorControlRequest::unified_gain(gain_type, value), 1)
                .await?;
            self.update_gain_cache_async(gain_type, value, value.max(1));
            return Ok(());
        }
        match gain_type {
            GainType::Lna => self.set_lna_gain_async(value).await,
            GainType::Mixer => self.set_mixer_gain_async(value).await,
            GainType::Vga => self.set_vga_gain_async(value).await,
            GainType::LnaAgc => self.set_lna_agc_async(value).await,
            GainType::MixerAgc => self.set_mixer_agc_async(value).await,
            GainType::Linearity => self.set_linearity_gain_async(value).await,
            GainType::Sensitivity => self.set_sensitivity_gain_async(value).await,
            GainType::Rf | GainType::Filter | GainType::RfAgc | GainType::FilterAgc => {
                Err(Error::Status(StatusCode::Unsupported))
            }
            GainType::Count => Err(Error::Status(StatusCode::InvalidParam)),
        }
    }

    /// Async counterpart to [`HydraSdr::set_linearity_gain`].
    pub async fn set_linearity_gain_async(&mut self, value: u8) -> Result<()> {
        let index = reverse_gain_table_index(value);
        self.set_mixer_agc_async(0).await?;
        self.set_lna_agc_async(0).await?;
        self.set_vga_gain_async(RFONE_LINEARITY_VGA_GAINS[index])
            .await?;
        self.set_mixer_gain_async(RFONE_LINEARITY_MIXER_GAINS[index])
            .await?;
        self.set_lna_gain_async(RFONE_LINEARITY_LNA_GAINS[index])
            .await?;
        self.update_gain_cache_async(GainType::Linearity, value.min(21), 21);
        Ok(())
    }

    /// Async counterpart to [`HydraSdr::set_sensitivity_gain`].
    pub async fn set_sensitivity_gain_async(&mut self, value: u8) -> Result<()> {
        let index = reverse_gain_table_index(value);
        self.set_mixer_agc_async(0).await?;
        self.set_lna_agc_async(0).await?;
        self.set_vga_gain_async(RFONE_SENSITIVITY_VGA_GAINS[index])
            .await?;
        self.set_mixer_gain_async(RFONE_SENSITIVITY_MIXER_GAINS[index])
            .await?;
        self.set_lna_gain_async(RFONE_SENSITIVITY_LNA_GAINS[index])
            .await?;
        self.update_gain_cache_async(GainType::Sensitivity, value.min(21), 21);
        Ok(())
    }

    /// Async counterpart to [`HydraSdr::set_rf_bias`].
    pub async fn set_rf_bias_async(&mut self, value: u8) -> Result<()> {
        self.control_out_async(VendorControlRequest::set_rf_bias(value))
            .await
    }

    /// Async counterpart to [`HydraSdr::set_packing`].
    pub async fn set_packing_async(&mut self, value: u8) -> Result<()> {
        self.control_in_min_async(VendorControlRequest::set_packing(value), 1)
            .await?;
        self.packing_enabled = value == 1;
        self.streaming.set_packing(self.packing_enabled)?;
        Ok(())
    }

    /// Async counterpart to [`HydraSdr::set_rf_port`].
    pub async fn set_rf_port_async(&mut self, port: RfPort) -> Result<()> {
        let response = self
            .control_in_min_async(VendorControlRequest::set_rf_port(port), 1)
            .await?;
        if response.first().copied() != Some(1) {
            return Err(Error::Status(StatusCode::InvalidParam));
        }
        Ok(())
    }

    /// Async counterpart to [`HydraSdr::receiver_mode`].
    pub async fn receiver_mode_async(&self, mode: ReceiverMode) -> Result<()> {
        self.control_out_async(VendorControlRequest::receiver_mode(mode))
            .await
    }

    /// Async counterpart to [`HydraSdr::stop_rx`].
    pub async fn stop_rx_async(&mut self) -> Result<()> {
        self.streaming.request_stop();
        if !self.reset_command {
            self.receiver_mode_async(ReceiverMode::Off).await?;
        }
        Ok(())
    }

    async fn sample_rate_config_async(&mut self, samplerate: u32) -> Result<(u32, u32, u32)> {
        if self.sample_rates.is_empty() {
            let _ = self.get_samplerates_async().await;
        }
        let (hardware_samplerate, decimation_factor) = self
            .sample_rate_hardware_config(samplerate)
            .unwrap_or((samplerate, 1));
        let rate_param = self.sample_rate_param_for_hardware_rate(hardware_samplerate)?;
        Ok((rate_param, hardware_samplerate, decimation_factor))
    }

    async fn bandwidth_param_async(&mut self, bandwidth: u32) -> Result<u32> {
        if self.bandwidths.is_empty() {
            let _ = self.get_bandwidths_async().await;
        }
        if let Some(index) = self.bandwidths.iter().position(|value| *value == bandwidth) {
            return Ok(index as u32);
        }
        if bandwidth >= MIN_BANDWIDTH_BY_VALUE {
            return Ok(bandwidth / MIN_BANDWIDTH_BY_VALUE);
        }
        if bandwidth < self.bandwidths.len() as u32 {
            return Ok(bandwidth);
        }
        Err(Error::Status(StatusCode::InvalidParam))
    }

    async fn set_legacy_gain_async(
        &mut self,
        gain_type: GainType,
        request: VendorRequest,
        value: u8,
        max_value: u8,
    ) -> Result<()> {
        let value = value.min(max_value);
        self.control_in_min_async(VendorControlRequest::legacy_gain(request, value), 1)
            .await?;
        self.update_gain_cache_async(gain_type, value, max_value);
        Ok(())
    }

    fn update_gain_cache_async(&mut self, gain_type: GainType, value: u8, max_value: u8) {
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

    async fn read_count_async(&self, request: VendorControlRequest) -> Result<u32> {
        let data = self.control_in_exact_async(request, 4).await?;
        Ok(u32::from_le_bytes(
            data[0..4].try_into().expect("four bytes"),
        ))
    }

    async fn read_u32_list_async(
        &self,
        request: VendorControlRequest,
        count: u32,
    ) -> Result<Vec<u32>> {
        let data = self
            .control_in_exact_async(request, count as usize * 4)
            .await?;
        decode_u32_le_words(&data)
    }

    async fn control_in_exact_async(
        &self,
        request: VendorControlRequest,
        len: usize,
    ) -> Result<Vec<u8>> {
        let data = self.control.control_in_async(request).await?;
        if data.len() < len {
            return Err(Error::Status(StatusCode::LibUsb));
        }
        Ok(data)
    }

    async fn control_in_min_async(
        &self,
        request: VendorControlRequest,
        len: usize,
    ) -> Result<Vec<u8>> {
        self.control_in_exact_async(request, len).await
    }

    async fn control_out_async(&self, request: VendorControlRequest) -> Result<()> {
        self.control.control_out_async(request).await
    }
}

impl<C> HydraSdr<C>
where
    C: ControlBackend + StreamingBackend,
{
    /// Start a persistent synchronous pull RX stream for raw USB blocks.
    pub fn start_raw_rx_stream(&mut self) -> Result<RawRxStream<C::BulkIn>> {
        self.receiver_mode(ReceiverMode::Off)?;
        self.receiver_mode(ReceiverMode::Rx)?;

        let bulk_in = match self.control.bulk_in(RFONE_RX_ENDPOINT) {
            Ok(bulk_in) => bulk_in,
            Err(err) => {
                let _ = self.receiver_mode(ReceiverMode::Off);
                return Err(err);
            }
        };

        match RawRxStream::start(bulk_in, self.streaming.config(), self.sample_type) {
            Ok(stream) => Ok(stream),
            Err(err) => {
                let _ = self.receiver_mode(ReceiverMode::Off);
                Err(err)
            }
        }
    }

    /// Stop a persistent synchronous raw RX stream and return its accumulated counters.
    pub fn stop_raw_rx_stream(&mut self, stream: RawRxStream<C::BulkIn>) -> Result<StreamingStats> {
        let (stats, stop_result) = self.close_raw_rx_stream(stream);
        stop_result?;
        Ok(stats)
    }

    pub(crate) fn close_raw_rx_stream(
        &mut self,
        mut stream: RawRxStream<C::BulkIn>,
    ) -> (StreamingStats, Result<()>) {
        let stats = stream.close();
        if !self.reset_command {
            return (stats, self.receiver_mode(ReceiverMode::Off));
        }
        (stats, Ok(()))
    }

    /// Start a persistent synchronous pull RX stream for unpacked float32 IQ samples.
    pub fn start_rx_stream(&mut self) -> Result<DirectRxStream<C::BulkIn>> {
        if self.sample_type != SampleType::Float32Iq || self.packing_enabled {
            return Err(Error::Status(StatusCode::Unsupported));
        }

        self.receiver_mode(ReceiverMode::Off)?;
        self.receiver_mode(ReceiverMode::Rx)?;

        let bulk_in = match self.control.bulk_in(RFONE_RX_ENDPOINT) {
            Ok(bulk_in) => bulk_in,
            Err(err) => {
                let _ = self.receiver_mode(ReceiverMode::Off);
                return Err(err);
            }
        };

        match DirectRxStream::start(bulk_in, self.streaming.config()) {
            Ok(stream) => Ok(stream),
            Err(err) => {
                let _ = self.receiver_mode(ReceiverMode::Off);
                Err(err)
            }
        }
    }

    /// Stop a persistent synchronous pull RX stream and return its accumulated counters.
    pub fn stop_rx_stream(&mut self, stream: DirectRxStream<C::BulkIn>) -> Result<StreamingStats> {
        let (stats, stop_result) = self.close_rx_stream(stream);
        stop_result?;
        Ok(stats)
    }

    pub(crate) fn close_rx_stream(
        &mut self,
        mut stream: DirectRxStream<C::BulkIn>,
    ) -> (StreamingStats, Result<()>) {
        let stats = stream.close();
        if !self.reset_command {
            return (stats, self.receiver_mode(ReceiverMode::Off));
        }
        (stats, Ok(()))
    }

    /// Start direct synchronous RX streaming.
    ///
    /// The callback contract mirrors C: each callback receives raw USB bytes and returning
    /// non-zero stops the stream. The method forces receiver OFF -> RX before streaming and runs
    /// stop cleanup afterwards.
    pub fn start_rx<F>(&mut self, callback: F) -> Result<StreamingStats>
    where
        F: FnMut(&Transfer<'_>) -> i32,
    {
        self.receiver_mode(ReceiverMode::Off)?;
        self.receiver_mode(ReceiverMode::Rx)?;

        let bulk_in = match self.control.bulk_in(RFONE_RX_ENDPOINT) {
            Ok(bulk_in) => bulk_in,
            Err(err) => {
                let _ = self.receiver_mode(ReceiverMode::Off);
                return Err(err);
            }
        };
        let stream_result = self.streaming.run(bulk_in, self.sample_type, callback);
        let stop_result = self.stop_rx();

        match (stream_result, stop_result) {
            (Err(err), _) => Err(err),
            (Ok(_), Err(err)) => Err(err),
            (Ok(stats), Ok(())) => Ok(stats),
        }
    }
}

impl<C> HydraSdr<C>
where
    C: AsyncControlBackend + AsyncStreamingBackend,
{
    /// Start a persistent async pull RX stream for raw USB blocks.
    pub async fn start_raw_rx_stream_async(&mut self) -> Result<AsyncRawRxStream<C::BulkIn>> {
        self.receiver_mode_async(ReceiverMode::Off).await?;
        self.receiver_mode_async(ReceiverMode::Rx).await?;

        let bulk_in = match self.control.bulk_in_async(RFONE_RX_ENDPOINT).await {
            Ok(bulk_in) => bulk_in,
            Err(err) => {
                let _ = self.receiver_mode_async(ReceiverMode::Off).await;
                return Err(err);
            }
        };

        match AsyncRawRxStream::start(bulk_in, self.streaming.config(), self.sample_type).await {
            Ok(stream) => Ok(stream),
            Err(err) => {
                let _ = self.receiver_mode_async(ReceiverMode::Off).await;
                Err(err)
            }
        }
    }

    /// Stop a persistent async raw RX stream and return its accumulated counters.
    pub async fn stop_raw_rx_stream_async(
        &mut self,
        mut stream: AsyncRawRxStream<C::BulkIn>,
    ) -> Result<StreamingStats> {
        let stats = stream.close();
        if !self.reset_command {
            self.receiver_mode_async(ReceiverMode::Off).await?;
        }
        Ok(stats)
    }

    /// Async counterpart to [`HydraSdr::start_rx`].
    pub async fn start_rx_async<F>(&mut self, callback: F) -> Result<StreamingStats>
    where
        F: FnMut(&Transfer<'_>) -> i32,
    {
        self.receiver_mode_async(ReceiverMode::Off).await?;
        self.receiver_mode_async(ReceiverMode::Rx).await?;

        let bulk_in = match self.control.bulk_in_async(RFONE_RX_ENDPOINT).await {
            Ok(bulk_in) => bulk_in,
            Err(err) => {
                let _ = self.receiver_mode_async(ReceiverMode::Off).await;
                return Err(err);
            }
        };
        let stream_result = self
            .streaming
            .run_async(bulk_in, self.sample_type, callback)
            .await;
        let stop_result = self.stop_rx_async().await;

        match (stream_result, stop_result) {
            (Err(err), _) => Err(err),
            (Ok(_), Err(err)) => Err(err),
            (Ok(stats), Ok(())) => Ok(stats),
        }
    }
}

impl HydraSdr<NusbControl> {
    /// Open the first visible HydraSDR RFOne through `nusb`, matching `hydrasdr_open`.
    pub fn open() -> Result<Self> {
        Self::open_sn_internal(None)
    }

    /// Open a HydraSDR RFOne by parsed serial number, matching `hydrasdr_open_sn`.
    pub fn open_sn(serial: u64) -> Result<Self> {
        Self::open_sn_internal(Some(serial))
    }

    /// Async counterpart to [`HydraSdr::open`].
    pub async fn open_async() -> Result<Self> {
        Self::open_sn_internal_async(None).await
    }

    /// Async counterpart to [`HydraSdr::open_sn`].
    pub async fn open_sn_async(serial: u64) -> Result<Self> {
        Self::open_sn_internal_async(Some(serial)).await
    }

    async fn open_sn_internal_async(serial: Option<u64>) -> Result<Self> {
        let info = discovery::select_nusb_device_async(serial).await?;
        let device = info.open().await.map_err(Error::from)?;
        match device.set_configuration(1).await {
            Ok(()) => {}
            Err(err) if err.kind() == nusb::ErrorKind::Unsupported => {}
            Err(err) => return Err(Error::from(err)),
        }
        let interface = device
            .detach_and_claim_interface(0)
            .await
            .map_err(Error::from)?;
        let dev = Self::from_control(NusbControl::new(device, interface));
        let firmware = dev.version_string_read_async().await?;
        if !firmware.starts_with(EXPECTED_FW_PREFIX) {
            return Err(Error::Status(StatusCode::NotFound));
        }
        Ok(dev)
    }

    fn open_sn_internal(serial: Option<u64>) -> Result<Self> {
        let info = discovery::select_nusb_device(serial)?;
        let device = info.open().wait().map_err(Error::from)?;
        match device.set_configuration(1).wait() {
            Ok(()) => {}
            Err(err) if err.kind() == nusb::ErrorKind::Unsupported => {}
            Err(err) => return Err(Error::from(err)),
        }
        let interface = device
            .detach_and_claim_interface(0)
            .wait()
            .map_err(Error::from)?;
        let dev = Self::from_control(NusbControl::new(device, interface));
        let firmware = dev.version_string_read()?;
        if !firmware.starts_with(EXPECTED_FW_PREFIX) {
            return Err(Error::Status(StatusCode::NotFound));
        }
        Ok(dev)
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

fn build_virtual_samplerates(hardware_rates: &[u32]) -> Vec<u32> {
    let mut rates = Vec::with_capacity(hardware_rates.len() * DECIMATION_FACTORS_ASC.len());
    for hardware_rate in hardware_rates {
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
