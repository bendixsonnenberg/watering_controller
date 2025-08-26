use embassy_sync::once_lock::*;
use embassy_time::Timer;
use embedded_io::Read;
use embedded_sdmmc::{BlockDevice, File, TimeSource};
use heapless::{String, Vec};
use log::{error, info};
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
                    Some('x')
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
    let mut buf = [0; 64];
    let _ = f.read(&mut buf);
    info!("read_settings: created iterator");
    Timer::after_millis(500).await;
    let mut vec = Vec::<u8, 64>::new();
    vec.extend_from_slice(&buf);
    vec.retain(|&c| c != 0);
    let mut s: String<64> = String::from_utf8(vec).unwrap();
    let mut p: String<64> = String::new();
    info!("read_settings: created string : {:?}", s);
    Timer::after_millis(500).await;

    while let c = s.remove(0) {
        if c == '\n' {
            break;
        }
        let _ = p.push(c);
    }
    let _ = SSID.init(p.clone());
    info!("read_settings: got ssid {}", SSID.get().await);
    Timer::after_millis(500).await;
    p.clear();
    while let c = s.remove(0) {
        if c == '\n' {
            break;
        }
        let _ = p.push(c);
    }
    let _ = PASSWORD.init(p.clone());
    info!("read_settings: got password {}", p);
    Timer::after_millis(500).await;
    //TODO: read sensor settings
}
