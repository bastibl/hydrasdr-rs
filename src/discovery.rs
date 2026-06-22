use crate::types::BoardId;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct UsbDeviceId {
    pub vid: u16,
    pub pid: u16,
    pub description: &'static str,
    pub board_id: BoardId,
}

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

pub fn parse_hydrasdr_serial(serial: &str) -> Option<u64> {
    let hex = serial.strip_prefix("HYDRASDR SN:")?.trim();
    if hex.len() != 16 || !hex.bytes().all(|b| b.is_ascii_hexdigit()) {
        return None;
    }
    u64::from_str_radix(hex, 16).ok()
}
