use embassy_futures::select;
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::peripherals::I2C1;
use embassy_rp::pio::Pio;
use embassy_time::Timer;
use menu::{MenuRunner, Sensor};

use heapless::String;
use lcd_lcm1602_i2c::async_lcd::Lcd;

use crate::can::{CanReceiver, CanSender, get_value, set_value};
use crate::{DisplayI2C, Irqs, MenuInput, SensorBitmap};
use can_contract::{CommandData, CommandDataContainer};
use core::fmt::Write;
use embassy_rp::i2c;
use embassy_rp::pio_programs::rotary_encoder::{Direction, PioEncoder, PioEncoderProgram};
use log::*;

const MOISTURE_INCREMENT_STEP_SIZE: u16 = 10;
const BACKOFF_TIME_INCREMENT_STEP_SIZE: u16 = 1;
const WATERING_TIME_INCREMENT_STEP_SIZE: u16 = 1;

const FOCUS_COLOR: CommandData = CommandData::Light(0b11111, 0, 0);
#[derive(Clone, Copy)]

struct SensorBuilder {
    sensors: &'static SensorBitmap,
    can_tx: CanSender,
    can_rx: CanReceiver,
}
impl SensorBuilder {
    fn lean_sensor_from_id(self, id: Option<usize>) -> Option<MenuSensor> {
        if let Some(id) = id {
            if self.sensors.lock(|map| map.borrow().get(id as usize)) {
                return Some(MenuSensor {
                    builder: self,
                    id: id as u8,
                    threshold: None,
                    moisture: None,
                    temperature: None,
                    humidity: None,
                    backoff_time: None,
                    watering_time: None,
                });
            }
        }
        // id = 0 shows that no sensor was found
        None
    }
    // try to get the threshold for this sensor, if it failes return none
    async fn populate(mut self, mut sensor: MenuSensor) -> Option<MenuSensor> {
        trace!(
            "populating with: id: {}, threshold: {:?}",
            sensor.id, sensor.threshold
        );
        if let CommandData::Sensors(moisture, temp, humidity) = get_value(
            &mut self.can_tx,
            &mut self.can_rx,
            CommandData::Sensors(None, None, None),
            sensor.id,
            &self.sensors,
        )
        .await?
        {
            sensor.moisture = moisture;
            sensor.temperature = temp;
            sensor.humidity = humidity;

            trace!("got moisture");
        };
        if let CommandData::Settings(threshold, backoff_time, watering_time) = get_value(
            &mut self.can_tx,
            &mut self.can_rx,
            CommandData::Settings(0, 0, 0),
            sensor.id,
            &self.sensors,
        )
        .await?
        {
            sensor.threshold = Some(threshold);
            sensor.backoff_time = Some(backoff_time);
            sensor.watering_time = Some(watering_time);
            info!("Got threshold, backoff_time, watering_time");
        };
        Some(sensor)
    }
    fn focus(mut self, sensor: MenuSensor) {
        set_value(
            &mut self.can_tx,
            CommandDataContainer::Data {
                target_id: sensor.id,
                src_id: 0,
                data: FOCUS_COLOR,
            },
        );
    }
    fn unfocus(mut self, sensor: MenuSensor) {
        set_value(
            &mut self.can_tx,
            CommandDataContainer::Data {
                target_id: sensor.id,
                src_id: 0,
                data: CommandData::LightOff,
            },
        );
    }
}
impl menu::SensorBuilder for SensorBuilder {
    async fn party(mut self) {
        self.sensors.lock(|sensors| {
            for sensor in sensors.borrow().into_iter() {
                set_value(
                    &mut self.can_tx,
                    CommandDataContainer::Data {
                        target_id: sensor as u8,
                        src_id: 0,
                        data: CommandData::LightRandom,
                    },
                );
            }
        })
    }
    async fn unparty(mut self) {
        self.sensors.lock(|sensors| {
            for sensor in sensors.borrow().into_iter() {
                set_value(
                    &mut self.can_tx,
                    CommandDataContainer::Data {
                        target_id: sensor as u8,
                        src_id: 0,
                        data: CommandData::LightOff,
                    },
                );
            }
        })
    }
}
#[derive(Clone)]
struct MenuSensor {
    #[allow(dead_code)]
    builder: SensorBuilder,
    id: u8,
    threshold: Option<u16>,
    moisture: Option<u16>,
    temperature: Option<i16>,
    humidity: Option<u16>,
    backoff_time: Option<u16>,
    watering_time: Option<u16>,
}
impl Sensor<SensorBuilder> for MenuSensor {
    fn first(builder: SensorBuilder) -> Option<Self> {
        let id = builder.sensors.lock(|cell| cell.borrow().first_index());
        builder.lean_sensor_from_id(id)
    }
    fn next(self, builder: SensorBuilder) -> Option<Self> {
        let id = builder
            .sensors
            .lock(|cell| cell.borrow().next_index(self.id as usize));
        match id {
            Some(id) => builder.lean_sensor_from_id(Some(id)),
            None => Sensor::first(builder),
        }
    }
    async fn populate(self, builder: SensorBuilder) -> Option<Self> {
        builder.populate(self).await
    }
    async fn focus(self, builder: SensorBuilder) {
        builder.focus(self);
    }
    async fn unfocus(self, builder: SensorBuilder) {
        builder.unfocus(self);
    }

    fn prev(self, builder: SensorBuilder) -> Option<Self> {
        let id = builder.sensors.lock(|cell| {
            let mut id = cell.borrow().prev_index(self.id as usize);
            if id.is_none() {
                id = cell.borrow().last_index();
            }
            id
        });
        builder.lean_sensor_from_id(id)
    }
    fn get_threshold(&self) -> u16 {
        self.threshold.unwrap_or(0)
    }
    fn get_backoff_time(&self) -> u16 {
        self.backoff_time.unwrap_or(0)
    }
    fn get_watering_time(&self) -> u16 {
        self.watering_time.unwrap_or(0)
    }
    fn get_id(&self) -> u8 {
        self.id
    }
    fn get_moisture(&self) -> u16 {
        self.moisture.unwrap_or(0)
    }
    fn get_temperature(&self) -> Option<i16> {
        self.temperature
    }
    fn get_humidity(&self) -> Option<u16> {
        self.humidity
    }
    fn increase_threshold(mut self, mut _builder: SensorBuilder) -> Option<Self> {
        trace!("increase threshold");
        if let Some(threshold) = self.threshold {
            self.threshold = Some(threshold.saturating_add(MOISTURE_INCREMENT_STEP_SIZE));
        } else {
            self.threshold = None
        }
        self.send_over_can();
        Some(self)
    }
    fn decrease_threshold(mut self, mut _builder: SensorBuilder) -> Option<Self> {
        trace!("decrease threshold");
        if let Some(threshold) = self.threshold {
            self.threshold = Some(threshold.saturating_sub(MOISTURE_INCREMENT_STEP_SIZE));
        } else {
            self.threshold = None
        }
        self.send_over_can();
        Some(self)
    }
    fn increase_backoff_time(mut self, mut _builder: SensorBuilder) -> Option<Self> {
        trace!("increase backoff_time");
        if let Some(backoff_time) = self.backoff_time {
            self.backoff_time = Some(backoff_time.saturating_add(BACKOFF_TIME_INCREMENT_STEP_SIZE));
        } else {
            self.backoff_time = None
        }
        self.send_over_can();
        Some(self)
    }
    fn decrease_backoff_time(mut self, mut _builder: SensorBuilder) -> Option<Self> {
        trace!("decrease backoff_time");
        if let Some(backoff_time) = self.backoff_time {
            self.backoff_time = Some(backoff_time.saturating_sub(BACKOFF_TIME_INCREMENT_STEP_SIZE));
        } else {
            self.backoff_time = None
        }
        self.send_over_can();
        Some(self)
    }
    fn increase_watering_time(mut self, mut _builder: SensorBuilder) -> Option<Self> {
        trace!("increase watering_time");
        if let Some(watering_time) = self.watering_time {
            self.watering_time =
                Some(watering_time.saturating_add(WATERING_TIME_INCREMENT_STEP_SIZE));
        } else {
            self.watering_time = None
        }
        self.send_over_can();
        Some(self)
    }
    fn decrease_watering_time(mut self, mut _builder: SensorBuilder) -> Option<Self> {
        trace!("decrease watering_time");
        if let Some(watering_time) = self.watering_time {
            self.watering_time =
                Some(watering_time.saturating_sub(WATERING_TIME_INCREMENT_STEP_SIZE));
        } else {
            self.watering_time = None
        }
        self.send_over_can();
        Some(self)
    }
}

impl MenuSensor {
    fn send_over_can(&mut self) {
        if let (Some(threshold), Some(watering_time), Some(backoff_time)) =
            (self.threshold, self.watering_time, self.backoff_time)
        {
            let mut tx = self.builder.can_tx;
            set_value(
                &mut tx,
                CommandDataContainer::Data {
                    target_id: self.id,
                    src_id: 0,
                    data: CommandData::Settings(threshold, backoff_time, watering_time),
                },
            )
        }
    }
}
async fn write_two_lines(
    lcd: &mut Lcd<'_, i2c::I2c<'_, I2C1, i2c::Async>, embassy_time::Delay>,
    text: &str,
) {
    let _ = lcd.clear().await;
    let _ = lcd.set_cursor(0, 0).await;
    let (first, second) = match text.split_once(|c| c == '\n') {
        Some((first, second)) => (first, second),
        None => (text, ""),
    };
    match lcd.write_str(first).await {
        Ok(_) => {}
        Err(e) => {
            info!("{:?}", e);
        }
    }
    let _ = lcd.set_cursor(1, 0).await;
    match lcd.write_str(second).await {
        Ok(_) => {}
        Err(e) => {
            info!("{:?}", e);
        }
    }
}
#[embassy_executor::task]
pub async fn menu_handle(
    can_tx: CanSender,
    can_rx: CanReceiver,
    sensors: &'static SensorBitmap,
    display_resources: DisplayI2C,
    menu_resources: MenuInput,
) {
    const ADDR: u8 = 0x27;
    let mut runner: MenuRunner<_, SensorBuilder> =
        MenuRunner::<MenuSensor, SensorBuilder>::new(SensorBuilder {
            sensors,
            can_tx,
            can_rx,
        });
    let mut i2c = i2c::I2c::new_async(
        display_resources.i2c,
        display_resources.sdc,
        display_resources.sda,
        Irqs,
        i2c::Config::default(),
    );
    info!("created i2c");
    let mut delay = embassy_time::Delay;
    let mut lcd = match lcd_lcm1602_i2c::async_lcd::Lcd::new(&mut i2c, &mut delay)
        .with_address(ADDR)
        .with_rows(2)
        .with_cursor_on(false)
        .init()
        .await
    {
        Ok(lcd) => lcd,
        Err(e) => {
            error!("failed at init of i2c: {:?}", e);
            Timer::after_millis(50).await;
            return;
        }
    };
    let mut back_button = Input::new(menu_resources.back, Pull::Up);
    let mut enter_button = Input::new(menu_resources.enter, Pull::Down);

    let Pio {
        mut common, sm0, ..
    } = Pio::new(menu_resources.pio, Irqs);
    let prg = PioEncoderProgram::new(&mut common);
    let mut encoder = PioEncoder::new(
        &mut common,
        sm0,
        menu_resources.encoder_clock,
        menu_resources.encoder_dt,
        &prg,
    );

    loop {
        runner.update().await;
        let mut buffer: String<128> = String::new();
        let Ok(_) = core::writeln!(&mut buffer, "{}", runner) else {
            break;
        };
        write_two_lines(&mut lcd, buffer.as_str()).await;
        match embassy_futures::select::select3(
            back_button.wait_for_falling_edge(),
            enter_button.wait_for_falling_edge(),
            encoder.read(),
        )
        .await
        {
            select::Either3::First(_) => {
                info!("back");
                runner.input(menu::Input::Back).await;
            }
            select::Either3::Second(_) => {
                info!("enter");
                runner.input(menu::Input::Enter).await;
            }
            select::Either3::Third(Direction::Clockwise) => {
                info!("left");
                runner.input(menu::Input::Left).await;
            }
            select::Either3::Third(Direction::CounterClockwise) => {
                info!("right");
                runner.input(menu::Input::Right).await;
            }
        }
    }
}
