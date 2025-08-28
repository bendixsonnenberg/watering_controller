use embassy_sync::once_lock::*;
use embassy_time::Timer;
use embedded_sdmmc::{BlockDevice, File, TimeSource};
use heapless::{String, Vec};
use log::info;
use serde::Deserialize;

use crate::can::{CanSender, set_value};
pub static SSID: OnceLock<String<64>> = OnceLock::new();
pub static PASSWORD: OnceLock<String<64>> = OnceLock::new();
pub static READ_BUF: usize = 20_000;

#[allow(dead_code)]
#[derive(Deserialize)]
pub struct SensorSettings {
    id: u8,
    threshold: u16,
    watering_time: u16,
    backoff_time: u16,
}

pub async fn read_sensor_settings<
    BD: BlockDevice,
    TS: TimeSource,
    const MDIR: usize,
    const MFILE: usize,
    const MVOL: usize,
>(
    f: File<'_, BD, TS, MDIR, MFILE, MVOL>,
    can_tx: &mut CanSender,
) {
    let Some(s): Option<String<READ_BUF>> = read_file_into_string(f) else {
        return;
    };
    let sensors = serde_json_core::from_str::<Vec<SensorSettings, 255>>(&s);

    Timer::after_secs(20).await;
    if sensors.is_err() {
        return;
    }
    let sensors = sensors.expect("checked for err").0;
    for s in sensors {
        set_value(
            can_tx,
            can_contract::CommandDataContainer::Data {
                target_id: s.id,
                src_id: 0,
                data: can_contract::CommandData::Settings(
                    s.threshold,
                    s.backoff_time,
                    s.watering_time,
                ),
            },
        );
        // allow the can bus to actually send all remaining settings out before adding new
        // we can take our time, since we are only delaying the logging part
        Timer::after_millis(50).await;
    }
}
fn read_file_into_string<
    const N: usize,
    BD: BlockDevice,
    TS: TimeSource,
    const MDIR: usize,
    const MFILE: usize,
    const MVOL: usize,
>(
    f: File<'_, BD, TS, MDIR, MFILE, MVOL>,
) -> Option<String<N>> {
    let mut buf = [0; N];
    let _ = f.read(&mut buf);
    let mut vec = Vec::<u8, N>::new();
    let _ = vec.extend_from_slice(&buf);
    vec.retain(|&c| c != 0);
    if let Ok(string) = String::from_utf8(vec) {
        Some(string)
    } else {
        None
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
    let Some(mut s): Option<String<64>> = read_file_into_string(f) else {
        return;
    };
    let mut p: String<64> = String::new();
    info!("read_settings: created string : {:?}", s);
    Timer::after_millis(500).await;

    loop {
        let c = s.remove(0);
        if c == '\n' {
            break;
        }
        let _ = p.push(c);
    }
    let _ = SSID.init(p.clone());
    info!("read_settings: got ssid {}", SSID.get().await);
    Timer::after_millis(500).await;
    p.clear();
    loop {
        let c = s.remove(0);
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
