//! USB discovery helpers for HydraSDR RFOne devices.

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
    /// Parsed 64-bit RFOne serial number.
    ///
    /// This is `0` when the USB serial string is absent or does not use the
    /// expected HydraSDR format.
    pub serial: u64,
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
            serial: normalized_serial(info.serial_number()),
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

/// List visible HydraSDR devices synchronously with `.wait()` or asynchronously
/// with `.await`.
pub(crate) fn list_devices() -> impl MaybeFuture<Output = Result<Vec<DeviceDescriptor>>> {
    nusb::list_devices().map(|devices| {
        Ok(devices
            .map_err(Error::from)?
            .filter_map(|device| DeviceDescriptor::from_nusb(&device))
            .collect())
    })
}

pub(crate) fn select_nusb_device(
    serial: Option<u64>,
) -> impl MaybeFuture<Output = Result<nusb::DeviceInfo>> {
    nusb::list_devices().map(move |devices| {
        devices
            .map_err(Error::from)?
            .find(|device| matches_device(device, serial))
            .ok_or(Error::DeviceNotFound)
    })
}

#[cfg(target_arch = "wasm32")]
async fn request_nusb_device(serial: Option<u64>) -> Result<Option<nusb::DeviceInfo>> {
    let selectors = USB_DEVICE_IDS
        .iter()
        .map(|device_id| {
            let selector = nusb::DeviceSelector::all().with_vid_pid(device_id.vid, device_id.pid);
            if let Some(serial) = serial.filter(|serial| *serial != 0) {
                selector.with_serial_number(format!("HYDRASDR SN:{serial:016X}"))
            } else {
                selector
            }
        })
        .collect::<Vec<_>>();

    nusb::request_device(&selectors).await.map_err(Error::from)
}

/// Ask the browser to grant access to a matching HydraSDR without opening it.
#[cfg(target_arch = "wasm32")]
pub(crate) async fn request_device_permission(serial: Option<u64>) -> Result<()> {
    for device in nusb::list_devices().await.map_err(Error::from)? {
        if matches_device(&device, serial) {
            return Ok(());
        }
    }

    request_nusb_device(serial)
        .await?
        .filter(|device| matches_device(device, serial))
        .map(|_| ())
        .ok_or(Error::DeviceNotFound)
}

fn matches_device(device: &nusb::DeviceInfo, serial: Option<u64>) -> bool {
    find_usb_device_id(device.vendor_id(), device.product_id()).is_some()
        && serial.is_none_or(|wanted| normalized_serial(device.serial_number()) == wanted)
}

fn normalized_serial(serial: Option<&str>) -> u64 {
    serial.and_then(parse_hydrasdr_serial).unwrap_or(0)
}

/// Parse the C firmware serial string format `HYDRASDR SN:<16 hex digits>`.
pub(crate) fn parse_hydrasdr_serial(serial: &str) -> Option<u64> {
    let hex = serial.strip_prefix("HYDRASDR SN:")?.trim();
    if hex.len() != 16 || !hex.bytes().all(|b| b.is_ascii_hexdigit()) {
        return None;
    }
    u64::from_str_radix(hex, 16).ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn serial_normalization_reserves_zero_for_missing_or_invalid_values() {
        assert_eq!(normalized_serial(None), 0);
        assert_eq!(normalized_serial(Some("")), 0);
        assert_eq!(normalized_serial(Some("HYDRASDR SN:not-hex")), 0);
        assert_eq!(normalized_serial(Some("HYDRASDR SN:0000000000000000")), 0);
        assert_eq!(
            normalized_serial(Some("HYDRASDR SN:123456789ABCDEF0")),
            0x1234_5678_9abc_def0
        );
    }
}
