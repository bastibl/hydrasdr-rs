#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum BoardId {
    ProtoHydraSdr = 0,
    HydraSdrRfOneOfficial = 1,
    Invalid = 0xff,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct UnknownBoardId(pub u8);

impl TryFrom<u8> for BoardId {
    type Error = UnknownBoardId;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::ProtoHydraSdr),
            1 => Ok(Self::HydraSdrRfOneOfficial),
            0xff => Ok(Self::Invalid),
            other => Err(UnknownBoardId(other)),
        }
    }
}

pub fn board_id_name_raw(board_id: u8) -> &'static str {
    match BoardId::try_from(board_id) {
        Ok(BoardId::Invalid) => "Invalid Board ID",
        Ok(BoardId::ProtoHydraSdr) => "HydraSDR RFOne Legacy VID/PID",
        Ok(BoardId::HydraSdrRfOneOfficial) => "HydraSDR RFOne Official VID/PID",
        Err(_) => "Unknown Board ID",
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum SampleType {
    Float32Iq = 0,
    Float32Real = 1,
    Int16Iq = 2,
    Int16Real = 3,
    Uint16Real = 4,
    Raw = 5,
    Int8Iq = 6,
    Uint8Iq = 7,
    Int8Real = 8,
    Uint8Real = 9,
    End = 10,
}

pub const fn sample_type_bit(sample_type: SampleType) -> u16 {
    1u16 << (sample_type as u8)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum DecimationMode {
    LowBandwidth = 0,
    HighDefinition = 1,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LibVersion {
    pub major_version: u32,
    pub minor_version: u32,
    pub revision: u32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PartIdSerialNo {
    pub part_id: [u32; 2],
    pub serial_no: [u32; 4],
}
