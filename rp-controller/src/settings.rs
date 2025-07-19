use embassy_sync::once_lock::*;
use embedded_io::Read;
use embedded_sdmmc::{BlockDevice, File, TimeSource};
use heapless::String;
use log::error;
pub static SSID: OnceLock<String<64>> = OnceLock::new();
pub static PASSWORD: OnceLock<String<64>> = OnceLock::new();

pub struct SensorSettings {
    threshold: u16,
    watering_time: u16,
    backoff_time: u16,
}

pub struct FileIterator<R: Read, const N: usize> {
    pub f: R,
    pub buf: [u8; N],
    pub offset: usize,
}

impl<R: Read, const N: usize> FileIterator<R, N> {
    fn next(&mut self) -> Option<char> {
        match self.buf.len() - self.offset {
            0 => match self.f.read(&mut self.buf) {
                Ok(0) => None,
                Ok(_count) => {
                    self.offset = 0;
                    self.next()
                }
                Err(e) => {
                    error!("{:?}", e);
                    None
                }
            },
            _ => {
                self.offset += 1;
                Some(self.buf[self.offset - 1] as char)
            }
        }
    }
}
pub async fn read_settings<
    BD: BlockDevice,
    TS: TimeSource,
    const MDIR: usize,
    const MFILE: usize,
    const MVOL: usize,
>(
    f: File<'_, BD, TS, MDIR, MFILE, MVOL>,
) -> () {
    // read WIFI Settings
    let mut iterator = FileIterator {
        f: f,
        buf: [0; 64],
        offset: 0,
    };
    let mut s: String<64> = String::new();
    while let Some(c) = iterator.next() {
        if c == '\n' {
            break;
        }
        let _ = s.push(c);
    }
    let _ = SSID.init(s.clone());
    s.clear();
    while let Some(c) = iterator.next() {
        if c == '\n' {
            break;
        }
        let _ = s.push(c);
    }
    let _ = PASSWORD.init(s.clone());
    //TODO: read sensor settings
}
