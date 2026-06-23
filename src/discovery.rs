//! USB discovery helpers for HydraSDR RFOne devices.

use nusb::MaybeFuture;

use crate::errors::{Error, Result, StatusCode};
use crate::types::BoardId;

/// Known HydraSDR USB VID/PID pair and its board identity.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct UsbDeviceId {
    pub vid: u16,
    pub pid: u16,
    pub description: &'static str,
    pub board_id: BoardId,
}

/// Known HydraSDR RFOne USB IDs accepted by the direct open/list helpers.
pub const USB_DEVICE_IDS: &[UsbDeviceId] = &[
    UsbDeviceId {
        vid: 0x1d50,
        pid: 0x60a1,
        description: "HydraSDR RFOne Legacy VID/PID",
        board_id: BoardId::ProtoHydraSdr,
    },
    UsbDeviceId {
        vid: 0x38af,
        pid: 0x0001,
        description: "HydraSDR RFOne Official VID/PID",
        board_id: BoardId::HydraSdrRfOneOfficial,
    },
];

/// Device information collected from `nusb` without opening the interface.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct HydraSdrDeviceInfo {
    pub vid: u16,
    pub pid: u16,
    pub description: &'static str,
    pub board_id: BoardId,
    pub serial: Option<u64>,
    pub product_string: Option<String>,
}

impl HydraSdrDeviceInfo {
    /// Convert a `nusb` descriptor into direct HydraSDR metadata if the VID/PID matches.
    pub fn from_nusb(info: &nusb::DeviceInfo) -> Option<Self> {
        let device_id = find_usb_device_id(info.vendor_id(), info.product_id())?;
        Some(Self {
            vid: device_id.vid,
            pid: device_id.pid,
            description: device_id.description,
            board_id: device_id.board_id,
            serial: info.serial_number().and_then(parse_hydrasdr_serial),
            product_string: info.product_string().map(str::to_owned),
        })
    }
}

/// Find a known HydraSDR USB ID by VID/PID.
pub fn find_usb_device_id(vid: u16, pid: u16) -> Option<UsbDeviceId> {
    USB_DEVICE_IDS
        .iter()
        .copied()
        .find(|candidate| candidate.vid == vid && candidate.pid == pid)
}

/// List visible HydraSDR devices synchronously using `nusb::MaybeFuture::wait()`.
pub fn list_devices() -> Result<Vec<HydraSdrDeviceInfo>> {
    let devices = nusb::list_devices().wait().map_err(Error::from)?;
    Ok(devices
        .filter_map(|device| HydraSdrDeviceInfo::from_nusb(&device))
        .collect())
}

/// List visible HydraSDR devices through the async `nusb` path.
pub async fn list_devices_async() -> Result<Vec<HydraSdrDeviceInfo>> {
    let devices = nusb::list_devices().await.map_err(Error::from)?;
    Ok(devices
        .filter_map(|device| HydraSdrDeviceInfo::from_nusb(&device))
        .collect())
}

/// Return parsed serial numbers for visible HydraSDR devices.
pub fn list_device_serials() -> Result<Vec<u64>> {
    Ok(list_devices()?
        .into_iter()
        .filter_map(|d| d.serial)
        .collect())
}

/// Return parsed serial numbers for visible HydraSDR devices through the async path.
pub async fn list_device_serials_async() -> Result<Vec<u64>> {
    Ok(list_devices_async()
        .await?
        .into_iter()
        .filter_map(|d| d.serial)
        .collect())
}

pub(crate) fn select_nusb_device(serial: Option<u64>) -> Result<nusb::DeviceInfo> {
    for device in nusb::list_devices().wait().map_err(Error::from)? {
        if find_usb_device_id(device.vendor_id(), device.product_id()).is_none() {
            continue;
        }
        if let Some(wanted) = serial
            && device.serial_number().and_then(parse_hydrasdr_serial) != Some(wanted)
        {
            continue;
        }
        return Ok(device);
    }
    Err(Error::Status(StatusCode::NotFound))
}

pub(crate) async fn select_nusb_device_async(serial: Option<u64>) -> Result<nusb::DeviceInfo> {
    for device in nusb::list_devices().await.map_err(Error::from)? {
        if find_usb_device_id(device.vendor_id(), device.product_id()).is_none() {
            continue;
        }
        if let Some(wanted) = serial
            && device.serial_number().and_then(parse_hydrasdr_serial) != Some(wanted)
        {
            continue;
        }
        return Ok(device);
    }
    Err(Error::Status(StatusCode::NotFound))
}

/// Parse the C firmware serial string format `HYDRASDR SN:<16 hex digits>`.
pub fn parse_hydrasdr_serial(serial: &str) -> Option<u64> {
    let hex = serial.strip_prefix("HYDRASDR SN:")?.trim();
    if hex.len() != 16 || !hex.bytes().all(|b| b.is_ascii_hexdigit()) {
        return None;
    }
    u64::from_str_radix(hex, 16).ok()
}
