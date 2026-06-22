use nusb::MaybeFuture;

use crate::commands::{Capability, GainType, ReceiverMode, RfPort, VendorRequest};
use crate::discovery;
use crate::errors::{Error, Result, StatusCode};
use crate::rfone::{
    RFONE_HARDCODED_CAPS, RFONE_LINEARITY_LNA_GAINS, RFONE_LINEARITY_MIXER_GAINS,
    RFONE_LINEARITY_VGA_GAINS, RFONE_LNA_MAX_GAIN, RFONE_MAX_FREQ_HZ, RFONE_MIN_FREQ_HZ,
    RFONE_MIXER_MAX_GAIN, RFONE_SAMPLE_TYPES, RFONE_SENSITIVITY_LNA_GAINS,
    RFONE_SENSITIVITY_MIXER_GAINS, RFONE_SENSITIVITY_VGA_GAINS, RFONE_TYPICAL_POWER_MW,
    RFONE_VGA_MAX_GAIN, component_infos, default_gain_infos, rf_port_infos,
};
use crate::types::{BoardId, DeviceInfo, GainInfo, PartIdSerialNo, SampleType, Temperature};
use crate::usb::control::{
    ControlBackend, NusbControl, VendorControlRequest, decode_part_id_serial, decode_u32_le_words,
};

const EXPECTED_FW_PREFIX: &str = "HydraSDR RF";
const VERSION_STRING_SIZE: usize = 255;
const MIN_SAMPLERATE_BY_VALUE: u32 = 10_000;
const MIN_BANDWIDTH_BY_VALUE: u32 = 1_000;
const MAX_FREQ_HZ: u64 = 10_000_000_000;

#[derive(Debug)]
pub struct HydraSdr<C = NusbControl> {
    control: C,
    sample_type: SampleType,
    sample_rates: Vec<u32>,
    bandwidths: Vec<u32>,
    features: Option<u32>,
    gains: Vec<GainInfo>,
    current_samplerate: u32,
    current_bandwidth: u32,
    packing_enabled: bool,
    reset_command: bool,
}

impl<C: ControlBackend> HydraSdr<C> {
    pub fn from_control(control: C) -> Self {
        Self {
            control,
            sample_type: SampleType::Float32Iq,
            sample_rates: Vec::new(),
            bandwidths: Vec::new(),
            features: None,
            gains: default_gain_infos(),
            current_samplerate: 0,
            current_bandwidth: 0,
            packing_enabled: false,
            reset_command: false,
        }
    }

    pub fn control(&self) -> &C {
        &self.control
    }

    pub fn close(self) -> Result<()> {
        Ok(())
    }

    pub fn board_id_read(&self) -> Result<BoardId> {
        let data = self.control_in_exact(VendorControlRequest::board_id_read(), 1)?;
        BoardId::try_from(data[0]).map_err(|_| Error::Status(StatusCode::Other))
    }

    pub fn version_string_read(&self) -> Result<String> {
        let data = self.control_in_min(
            VendorControlRequest::version_string_read(VERSION_STRING_SIZE),
            0,
        )?;
        Ok(decode_c_string(&data))
    }

    pub fn board_partid_serialno_read(&self) -> Result<PartIdSerialNo> {
        let data = self.control_in_exact(VendorControlRequest::board_partid_serialno_read(), 24)?;
        decode_part_id_serial(&data)
    }

    pub fn get_capabilities(&self) -> Result<u32> {
        match self.control_in_exact(VendorControlRequest::get_capabilities(0), 4) {
            Ok(data) => Ok(u32::from_le_bytes(
                data[0..4].try_into().expect("four bytes"),
            )),
            Err(_) => Ok(RFONE_HARDCODED_CAPS),
        }
    }

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

    pub fn get_device_info(&mut self) -> Result<DeviceInfo> {
        let board_id = self.board_id_read()?;
        let firmware_version = self.version_string_read()?;
        let part_serial = self.board_partid_serialno_read()?;
        let features = self.get_capabilities()?;
        self.features = Some(features);
        Ok(DeviceInfo {
            board_id,
            board_name: "HydraSDR RFOne",
            firmware_version,
            part_serial,
            features,
            features_reserved: self.get_capabilities_reserved()?,
            gains: self.gains.clone(),
            components: component_infos(),
            min_frequency: RFONE_MIN_FREQ_HZ,
            max_frequency: RFONE_MAX_FREQ_HZ,
            rf_ports: rf_port_infos(),
            gpio_count: crate::rfone::RFONE_GPIO_COUNT,
            sample_types: RFONE_SAMPLE_TYPES,
            typical_power_mw: RFONE_TYPICAL_POWER_MW,
            max_power_mw: crate::rfone::RFONE_MAX_POWER_MW,
            max_safe_temp_celsius: crate::rfone::RFONE_MAX_SAFE_TEMP_C,
            current_samplerate: self.current_samplerate,
            current_bandwidth: self.current_bandwidth,
            current_sample_type: self.sample_type,
            current_packing: self.packing_enabled,
        })
    }

    pub fn get_samplerates(&mut self) -> Result<Vec<u32>> {
        let count = self.read_count(VendorControlRequest::get_samplerates_count(false))?;
        let rates =
            self.read_u32_list(VendorControlRequest::get_samplerates(count, false), count)?;
        self.sample_rates = rates.clone();
        Ok(rates)
    }

    pub fn set_samplerate(&mut self, samplerate: u32) -> Result<()> {
        let rate_param = self.sample_rate_param(samplerate)?;
        self.control_in_min(VendorControlRequest::set_samplerate(rate_param, 1), 1)?;
        self.current_samplerate = samplerate;
        Ok(())
    }

    pub fn get_bandwidths(&mut self) -> Result<Vec<u32>> {
        let count = self.read_count(VendorControlRequest::get_bandwidths_count())?;
        let bandwidths = self.read_u32_list(VendorControlRequest::get_bandwidths(count), count)?;
        self.bandwidths = bandwidths.clone();
        Ok(bandwidths)
    }

    pub fn set_bandwidth(&mut self, bandwidth: u32) -> Result<()> {
        let bandwidth_param = self.bandwidth_param(bandwidth)?;
        self.control_in_min(VendorControlRequest::set_bandwidth(bandwidth_param), 1)?;
        self.current_bandwidth = bandwidth;
        Ok(())
    }

    pub fn set_freq(&mut self, freq_hz: u64) -> Result<()> {
        if freq_hz == 0 || freq_hz > MAX_FREQ_HZ {
            return Err(Error::Status(StatusCode::InvalidParam));
        }
        self.control_out(VendorControlRequest::set_frequency(freq_hz))
    }

    pub fn set_lna_gain(&mut self, value: u8) -> Result<()> {
        self.set_legacy_gain(
            GainType::Lna,
            VendorRequest::SetLnaGain,
            value,
            RFONE_LNA_MAX_GAIN,
        )
    }

    pub fn set_mixer_gain(&mut self, value: u8) -> Result<()> {
        self.set_legacy_gain(
            GainType::Mixer,
            VendorRequest::SetMixerGain,
            value,
            RFONE_MIXER_MAX_GAIN,
        )
    }

    pub fn set_vga_gain(&mut self, value: u8) -> Result<()> {
        self.set_legacy_gain(
            GainType::Vga,
            VendorRequest::SetVgaGain,
            value,
            RFONE_VGA_MAX_GAIN,
        )
    }

    pub fn set_lna_agc(&mut self, value: u8) -> Result<()> {
        self.set_legacy_gain(GainType::LnaAgc, VendorRequest::SetLnaAgc, value, 1)
    }

    pub fn set_mixer_agc(&mut self, value: u8) -> Result<()> {
        self.set_legacy_gain(GainType::MixerAgc, VendorRequest::SetMixerAgc, value, 1)
    }

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

    pub fn get_gain(&self, gain_type: GainType) -> Result<GainInfo> {
        self.gains
            .iter()
            .copied()
            .find(|gain| gain.gain_type == gain_type)
            .ok_or(Error::Status(StatusCode::Unsupported))
    }

    pub fn get_all_gains(&self) -> Result<Vec<GainInfo>> {
        Ok(self.gains.clone())
    }

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

    pub fn set_rf_bias(&mut self, value: u8) -> Result<()> {
        self.control_out(VendorControlRequest::set_rf_bias(value))
    }

    pub fn set_packing(&mut self, value: u8) -> Result<()> {
        self.control_in_min(VendorControlRequest::set_packing(value), 1)?;
        self.packing_enabled = value == 1;
        Ok(())
    }

    pub fn set_rf_port(&mut self, port: RfPort) -> Result<()> {
        let response = self.control_in_min(VendorControlRequest::set_rf_port(port), 1)?;
        if response.first().copied() != Some(1) {
            return Err(Error::Status(StatusCode::InvalidParam));
        }
        Ok(())
    }

    pub fn reset(&mut self) -> Result<()> {
        let _ = self.control.control_in(VendorControlRequest::reset());
        self.reset_command = true;
        Ok(())
    }

    pub fn set_sample_type(&mut self, sample_type: SampleType) -> Result<()> {
        if sample_type == SampleType::End {
            return Err(Error::Status(StatusCode::InvalidParam));
        }
        self.sample_type = sample_type;
        Ok(())
    }

    pub fn get_sample_type(&self) -> SampleType {
        self.sample_type
    }

    pub fn gpio_write(&self, port: u8, pin: u8, value: u8) -> Result<()> {
        self.control_out(VendorControlRequest::gpio_write(port, pin, value)?)
    }

    pub fn gpio_read(&self, port: u8, pin: u8) -> Result<u8> {
        let data = self.control_in_exact(VendorControlRequest::gpio_read(port, pin)?, 1)?;
        Ok(data[0])
    }

    pub fn gpiodir_write(&self, port: u8, pin: u8, value: u8) -> Result<()> {
        self.control_out(VendorControlRequest::gpiodir_write(port, pin, value)?)
    }

    pub fn gpiodir_read(&self, port: u8, pin: u8) -> Result<u8> {
        let data = self.control_in_exact(VendorControlRequest::gpiodir_read(port, pin)?, 1)?;
        Ok(data[0])
    }

    pub fn clockgen_write(&self, reg: u8, value: u8) -> Result<()> {
        self.control_out(VendorControlRequest::clockgen_write(reg, value))
    }

    pub fn clockgen_read(&self, reg: u8) -> Result<u8> {
        let data = self.control_in_exact(VendorControlRequest::clockgen_read(reg), 1)?;
        Ok(data[0])
    }

    pub fn rf_frontend_write(&self, reg: u16, value: u32) -> Result<()> {
        self.control_out(VendorControlRequest::rf_frontend_write(reg, value))
    }

    pub fn rf_frontend_read(&self, reg: u16) -> Result<u32> {
        let data = self.control_in_exact(VendorControlRequest::rf_frontend_read(reg), 1)?;
        Ok(data[0] as u32)
    }

    pub fn spiflash_erase(&self) -> Result<()> {
        self.control_out(VendorControlRequest::spiflash_erase())
    }

    pub fn spiflash_erase_sector(&self, sector: u16) -> Result<()> {
        self.control_out(VendorControlRequest::spiflash_erase_sector(sector))
    }

    pub fn spiflash_write(&self, addr: u32, data: &[u8]) -> Result<()> {
        self.control_out(VendorControlRequest::spiflash_write(addr, data)?)
    }

    pub fn spiflash_read(&self, addr: u32, len: u16) -> Result<Vec<u8>> {
        self.control_in_exact(
            VendorControlRequest::spiflash_read(addr, len)?,
            len as usize,
        )
    }

    pub fn get_temperature(&self) -> Result<Temperature> {
        Err(Error::Status(StatusCode::Unsupported))
    }

    pub fn receiver_mode(&self, mode: ReceiverMode) -> Result<()> {
        self.control_out(VendorControlRequest::receiver_mode(mode))
    }

    fn sample_rate_param(&mut self, samplerate: u32) -> Result<u32> {
        if self.sample_rates.is_empty() {
            let _ = self.get_samplerates();
        }
        if let Some(index) = self
            .sample_rates
            .iter()
            .position(|rate| *rate == samplerate)
        {
            return Ok(index as u32);
        }
        if samplerate < MIN_SAMPLERATE_BY_VALUE {
            return Err(Error::Status(StatusCode::InvalidParam));
        }
        let mut rate_param = samplerate;
        if matches!(
            self.sample_type,
            SampleType::Float32Iq | SampleType::Int16Iq | SampleType::Int8Iq | SampleType::Uint8Iq
        ) {
            rate_param = rate_param.saturating_mul(2);
        }
        Ok(rate_param / 1000)
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

impl HydraSdr<NusbControl> {
    pub fn open() -> Result<Self> {
        Self::open_sn_internal(None)
    }

    pub fn open_sn(serial: u64) -> Result<Self> {
        Self::open_sn_internal(Some(serial))
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
