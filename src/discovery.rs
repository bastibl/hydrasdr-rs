//! USB discovery helpers for HydraSDR RFOne devices.

#[cfg(not(target_arch = "wasm32"))]
use nusb::MaybeFuture;

use crate::errors::{Error, Result};
/// Known HydraSDR USB VID/PID pair and its board identity.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct UsbDeviceId {
    pub vid: u16,
    pub pid: u16,
    pub description: &'static str,
}

/// Known HydraSDR RFOne USB IDs accepted by the direct open/list helpers.
pub(crate) const USB_DEVICE_IDS: &[UsbDeviceId] = &[
    UsbDeviceId {
        vid: 0x1d50,
        pid: 0x60a1,
        description: "HydraSDR RFOne Legacy VID/PID",
    },
    UsbDeviceId {
        vid: 0x38af,
        pid: 0x0001,
        description: "HydraSDR RFOne Official VID/PID",
    },
];

/// Device information collected from `nusb` without opening the interface.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DeviceDescriptor {
    /// USB vendor ID.
    pub vid: u16,
    /// USB product ID.
    pub pid: u16,
    /// Static board description matched from the known VID/PID table.
    pub description: &'static str,
    /// Parsed 64-bit RFOne serial number, if the device reports one.
    pub serial: Option<u64>,
    /// USB product string, if the backend reports one.
    pub product_string: Option<String>,
}

impl DeviceDescriptor {
    /// Convert a `nusb` descriptor into direct HydraSDR metadata if the VID/PID matches.
    pub(crate) fn from_nusb(info: &nusb::DeviceInfo) -> Option<Self> {
        let device_id = find_usb_device_id(info.vendor_id(), info.product_id())?;
        Some(Self {
            vid: device_id.vid,
            pid: device_id.pid,
            description: device_id.description,
            serial: info.serial_number().and_then(parse_hydrasdr_serial),
            product_string: info.product_string().map(str::to_owned),
        })
    }
}

/// Find a known HydraSDR USB ID by VID/PID.
pub(crate) fn find_usb_device_id(vid: u16, pid: u16) -> Option<UsbDeviceId> {
    USB_DEVICE_IDS
        .iter()
        .copied()
        .find(|candidate| candidate.vid == vid && candidate.pid == pid)
}

/// List visible HydraSDR devices synchronously using `nusb::MaybeFuture::wait()`.
#[cfg(not(target_arch = "wasm32"))]
pub(crate) fn list_devices() -> Result<Vec<DeviceDescriptor>> {
    let devices = nusb::list_devices().wait().map_err(Error::from)?;
    Ok(devices
        .filter_map(|device| DeviceDescriptor::from_nusb(&device))
        .collect())
}

/// List visible HydraSDR devices through the async `nusb` path.
pub(crate) async fn list_devices_async() -> Result<Vec<DeviceDescriptor>> {
    let devices = nusb::list_devices().await.map_err(Error::from)?;
    Ok(devices
        .filter_map(|device| DeviceDescriptor::from_nusb(&device))
        .collect())
}

#[cfg(not(target_arch = "wasm32"))]
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
    Err(Error::DeviceNotFound)
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

    #[cfg(target_arch = "wasm32")]
    if let Some(device) = request_nusb_device_async(serial).await? {
        return Ok(device);
    }

    Err(Error::DeviceNotFound)
}

#[cfg(target_arch = "wasm32")]
async fn request_nusb_device_async(serial: Option<u64>) -> Result<Option<nusb::DeviceInfo>> {
    let selectors = USB_DEVICE_IDS
        .iter()
        .map(|device_id| {
            let selector = nusb::DeviceSelector::all().with_vid_pid(device_id.vid, device_id.pid);
            if let Some(serial) = serial {
                selector.with_serial_number(format!("HYDRASDR SN:{serial:016X}"))
            } else {
                selector
            }
        })
        .collect::<Vec<_>>();

    nusb::request_device(&selectors).await.map_err(Error::from)
}

/// Parse the C firmware serial string format `HYDRASDR SN:<16 hex digits>`.
pub(crate) fn parse_hydrasdr_serial(serial: &str) -> Option<u64> {
    let hex = serial.strip_prefix("HYDRASDR SN:")?.trim();
    if hex.len() != 16 || !hex.bytes().all(|b| b.is_ascii_hexdigit()) {
        return None;
    }
    u64::from_str_radix(hex, 16).ok()
}
