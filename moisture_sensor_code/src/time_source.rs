use embedded_sdmmc::{TimeSource, Timestamp};

pub struct dummyTimeSource {}
impl dummyTimeSource {
    pub fn dummy() -> Self {
        Self {}
    }
}
impl TimeSource for dummyTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        return Timestamp::from_calendar(1970, 1, 1, 0, 0, 0).unwrap();
    }
}
