use crate::errors::{Error, Result, StatusCode};

#[derive(Debug)]
pub struct HydraSdr {
    _private: (),
}

impl HydraSdr {
    pub fn open() -> Result<Self> {
        Err(Error::Status(StatusCode::Unsupported))
    }

    pub fn close(self) -> Result<()> {
        Ok(())
    }
}
