use crate::can::{CanReceiver, get_value};
use crate::error::report_error;
use crate::settings::read_settings;
use crate::{CanSender, SensorBitmap, SpiSdcard};
use can_contract::CommandData;
use core::fmt::Write;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::spi;
use embassy_rp::spi::Spi;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{SdCard, VolumeIdx, VolumeManager};
use heapless::String;
use log::*;

#[allow(dead_code)]
struct DummyTimesource();

impl embedded_sdmmc::TimeSource for DummyTimesource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}
#[embassy_executor::task]
pub async fn sd_card_log(
    mut can_tx: CanSender,
    mut can_rx: CanReceiver,
    sensors: &'static SensorBitmap,
    mut sd_card_resources: SpiSdcard,
) {
    // this overarching loop enables the sd card to be inserted later
    loop {
        let mut config = spi::Config::default();
        config.frequency = 400_000; // for initilization the frequency has to be below 400khz
        let spi = Spi::new_blocking(
            sd_card_resources.spi.reborrow(),
            sd_card_resources.clk.reborrow(),
            sd_card_resources.mosi.reborrow(),
            sd_card_resources.miso.reborrow(),
            config,
        );
        let cs = Output::new(sd_card_resources.cs.reborrow(), Level::High);
        let spi_dev = ExclusiveDevice::new_no_delay(spi, cs);

        let sdcard = SdCard::new(spi_dev, embassy_time::Delay);
        sdcard.mark_card_uninit();

        let volume_mgr = VolumeManager::new(sdcard, DummyTimesource());

        let Ok(vol0) = volume_mgr.open_volume(VolumeIdx(0)) else {
            error!("Failed opening volume, not logging anymore, retrying");
            continue;
        };
        info!("got volume");

        let Ok(root_dir) = vol0.open_root_dir() else {
            error!("Failed opening dir");
            continue;
        };
        info!("got dir");

        if let Ok(settings_file) =
            root_dir.open_file_in_dir("settings", embedded_sdmmc::Mode::ReadOnly)
        {
            info!("read_file for settings");
            read_settings(settings_file).await;
        } else {
            error!("failed to get settings file. Does it exists in the root dir?");
        }

        let Ok(log_file) = root_dir.open_file_in_dir(
            "log_file.csv",
            embedded_sdmmc::Mode::ReadWriteCreateOrAppend,
        ) else {
            error!("failed opening file");
            continue;
        };

        let mut ticker = Ticker::every(Duration::from_secs_floor(30));
        loop {
            let time = Instant::now().as_secs();
            // we need to drop can after using it
            let mut buf: String<32> = String::new();

            let Ok(_) = core::write!(&mut buf, "Time: {}, ", time) else {
                error!("failed writing time");
                break;
            };
            let Ok(_) = log_file.write(buf.as_bytes()) else {
                error!("failed at writing");
                break;
            };
            let mut failed = false;
            {
                let sensors_map = sensors.lock(|sensor| sensor.borrow().clone());
                let mut buffer: String<128> = String::new();
                for sensor in sensors_map.into_iter() {
                    buffer.clear();
                    let id = sensor as u8;
                    let Some(CommandData::Sensors(Some(moisture), temp, humidity)) = get_value(
                        &mut can_tx,
                        &mut can_rx,
                        CommandData::Sensors(None, None, None),
                        id,
                        sensors,
                    )
                    .await
                    else {
                        continue;
                    };
                    let Some(CommandData::Settings(threshold, _backoff_time, _watering_time)) =
                        get_value(
                            &mut can_tx,
                            &mut can_rx,
                            CommandData::Settings(0, 0, 0),
                            id,
                            sensors,
                        )
                        .await
                    else {
                        continue;
                    };
                    let Ok(_) = core::write!(
                        &mut buffer,
                        "Sen: {}, Thr: {}, Mo: {},",
                        id,
                        threshold,
                        moisture,
                    ) else {
                        failed = true;
                        break;
                    };
                    // if the sensor has temp and humidity print them to sd card
                    if let Some(temp) = temp {
                        if let Some(hum) = humidity {
                            if temp < 0 {
                                report_error(crate::error::Error::SubFreezing(id));
                            }
                            let _ = core::write!(&mut buffer, "Temp: {}, Hum: {},", temp, hum);
                        }
                    }
                    let Ok(_) = log_file.write(buffer.as_bytes()) else {
                        failed = true;
                        break;
                    };
                }
                if failed {
                    error!("failed writing to sd card");
                    break;
                }
                let Ok(_) = log_file.write("\n".as_bytes()) else {
                    error!("failed newline");
                    break;
                };
                let Ok(_) = log_file.flush() else {
                    error!("failed flushing");
                    break;
                };
            }
            ticker.next().await;
        }
        error!("failed at logging");
        Timer::after_secs(60).await;
    }
}
