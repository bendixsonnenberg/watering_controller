use embedded_sdmmc::{TimeSource, Timestamp};

pub struct DummyTimeSource {}
impl DummyTimeSource {
    pub fn dummy() -> Self {
        Self {}
    }
}
impl TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        return Timestamp::from_calendar(1970, 1, 1, 0, 0, 0).unwrap();
    }
}
