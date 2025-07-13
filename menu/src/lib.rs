#![no_std]

use core::fmt::Display;

/// this module creates a menu that can tbe interacted with by using 1 rotary encoder and 2 buttons
/// it requires a function to print to a screen and functions to handle sending values to sensors
/// this will not be an arbitrary menu, rather it will be
#[derive(Clone, Copy, Debug)]
pub enum Input {
    Enter,
    Back,
    Right,
    Left,
}
/// this enum contains the possible states that the menu can be in. Some states can have values,
/// and some State, Input tupels result in sideeffect, therefore this is not a true finite
/// deterministic state automata
#[derive(Clone, Copy)]
pub enum State<T> {
    SensorsSelect,
    Errors,
    SensorSelection(Option<T>),
    SensorSettingSelectionThreshold(Option<T>),
    SensorSettingSelectionBackoffTime(Option<T>),
    SensorSettingSelectionWateringTime(Option<T>),
    SensorSettingThreshold(Option<T>),
    SensorSettingBackoffTime(Option<T>),
    SensorSettingWateringTime(Option<T>),
}

pub trait Sensor<B>: Sized {
    fn first(builder: B) -> Option<Self>;
    fn next(self, builder: B) -> Option<Self>;
    fn prev(self, builder: B) -> Option<Self>;
    fn get_threshold(&self) -> u16;
    fn get_backoff_time(&self) -> u16;
    fn get_watering_time(&self) -> u16;
    fn get_moisture(&self) -> u16;
    fn get_temperature(&self) -> Option<i16>;
    fn get_humidity(&self) -> Option<u16>;
    fn get_id(&self) -> u8;
    fn increase_threshold(self, builder: B) -> Option<Self>;
    fn decrease_threshold(self, builder: B) -> Option<Self>;
    fn increase_backoff_time(self, builder: B) -> Option<Self>;
    fn decrease_backoff_time(self, builder: B) -> Option<Self>;
    fn increase_watering_time(self, builder: B) -> Option<Self>;
    fn decrease_watering_time(self, builder: B) -> Option<Self>;
    #[allow(async_fn_in_trait)]
    async fn populate(self, builder: B) -> Option<Self>;
    #[allow(async_fn_in_trait)]
    async fn focus(self, builder: B);
    #[allow(async_fn_in_trait)]
    async fn unfocus(self, builder: B);
}

pub struct MenuRunner<T: Sensor<B> + Clone, B> {
    state: State<T>,
    builder: B,
}

impl<T: Sensor<B> + Clone, B: Clone> MenuRunner<T, B> {
    pub fn new(builder: B) -> Self {
        Self {
            state: State::<T>::SensorsSelect,
            builder,
        }
    }
    pub async fn update(&mut self) {
        let state = match self.state.clone() {
            State::SensorSelection(Some(sensor)) => {
                State::SensorSelection(sensor.populate(self.builder.clone()).await)
            }
            State::SensorSettingThreshold(Some(sensor)) => {
                State::SensorSettingThreshold(sensor.populate(self.builder.clone()).await)
            }
            s => s,
        };
        self.state = state;
    }
    pub async fn input(&mut self, input: Input) {
        use Input::*;
        use State::*;
        self.state = match (self.state.clone(), input) {
            (Errors, Left) => SensorsSelect,
            (Errors, _) => Errors,
            (SensorsSelect, Right) => Errors,
            (SensorsSelect, Enter) => {
                let sensor: Option<T> = Sensor::first(self.builder.clone());
                if let Some(s) = sensor.clone() {
                    s.focus(self.builder.clone()).await;
                }
                SensorSelection(sensor)
            }
            (SensorsSelect, _) => SensorsSelect,
            (SensorSelection(Some(sensor)), Right) => {
                let next_sensor = sensor.clone().next(self.builder.clone());
                if let Some(s) = next_sensor.clone() {
                    sensor.unfocus(self.builder.clone()).await;
                    s.focus(self.builder.clone()).await;
                }
                SensorSelection(next_sensor)
            }
            (SensorSelection(Some(sensor)), Left) => {
                let next_sensor = sensor.clone().prev(self.builder.clone());
                if let Some(s) = next_sensor.clone() {
                    sensor.unfocus(self.builder.clone()).await;
                    s.focus(self.builder.clone()).await;
                }

                SensorSelection(next_sensor)
            }
            (SensorSelection(None), Left | Right) => SensorSelection(None),
            (SensorSelection(None), Back) => SensorsSelect,
            (SensorSelection(Some(sensor)), Back) => {
                sensor.unfocus(self.builder.clone()).await;
                SensorsSelect
            }
            // selection of the possible settings
            (SensorSelection(sensor), Enter) => SensorSettingSelectionThreshold(sensor),

            (SensorSettingSelectionThreshold(sensor), Left) => {
                SensorSettingSelectionThreshold(sensor)
            }
            (SensorSettingSelectionThreshold(sensor), Right) => {
                SensorSettingSelectionBackoffTime(sensor)
            }
            (SensorSettingSelectionBackoffTime(sensor), Left) => {
                SensorSettingSelectionThreshold(sensor)
            }
            (SensorSettingSelectionBackoffTime(sensor), Right) => {
                SensorSettingSelectionWateringTime(sensor)
            }
            (SensorSettingSelectionWateringTime(sensor), Left) => {
                SensorSettingSelectionBackoffTime(sensor)
            }
            (SensorSettingSelectionWateringTime(sensor), Right) => {
                SensorSettingSelectionWateringTime(sensor)
            }
            // getting back from setting selection
            (SensorSettingSelectionThreshold(sensor), Back) => SensorSelection(sensor),
            (SensorSettingSelectionBackoffTime(sensor), Back) => SensorSelection(sensor),
            (SensorSettingSelectionWateringTime(sensor), Back) => SensorSelection(sensor),

            // entering the settings
            (SensorSettingSelectionThreshold(sensor), Enter) => SensorSettingThreshold(sensor),
            (SensorSettingSelectionBackoffTime(sensor), Enter) => SensorSettingBackoffTime(sensor),
            (SensorSettingSelectionWateringTime(sensor), Enter) => {
                SensorSettingWateringTime(sensor)
            }
            // threshold setting
            (SensorSettingThreshold(Some(sensor)), Left) => {
                SensorSettingThreshold(sensor.increase_threshold(self.builder.clone()))
            }
            (SensorSettingThreshold(None), Left | Right) => SensorSettingThreshold(None),
            (SensorSettingThreshold(Some(sensor)), Right) => {
                SensorSettingThreshold(sensor.decrease_threshold(self.builder.clone()))
            }
            (SensorSettingThreshold(sensor), Back) => SensorSettingSelectionThreshold(sensor),
            (SensorSettingThreshold(sensor), Enter) => SensorSettingThreshold(sensor),
            // backoff setting
            (SensorSettingBackoffTime(Some(sensor)), Right) => {
                SensorSettingBackoffTime(sensor.increase_backoff_time(self.builder.clone()))
            }
            (SensorSettingBackoffTime(None), Left | Right) => SensorSettingBackoffTime(None),
            (SensorSettingBackoffTime(Some(sensor)), Left) => {
                SensorSettingBackoffTime(sensor.decrease_backoff_time(self.builder.clone()))
            }
            (SensorSettingBackoffTime(sensor), Back) => SensorSettingSelectionBackoffTime(sensor),
            (SensorSettingBackoffTime(sensor), Enter) => SensorSettingBackoffTime(sensor),
            // watering time setting
            (SensorSettingWateringTime(Some(sensor)), Right) => {
                SensorSettingWateringTime(sensor.increase_watering_time(self.builder.clone()))
            }
            (SensorSettingWateringTime(None), Left | Right) => SensorSettingWateringTime(None),
            (SensorSettingWateringTime(Some(sensor)), Left) => {
                SensorSettingWateringTime(sensor.decrease_watering_time(self.builder.clone()))
            }
            (SensorSettingWateringTime(sensor), Back) => SensorSettingSelectionWateringTime(sensor),
            (SensorSettingWateringTime(sensor), Enter) => SensorSettingWateringTime(sensor),
        }
    }
}
impl<T: Sensor<B> + Clone, B> Display for MenuRunner<T, B> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        use State::*;
        match self.state.clone() {
            SensorsSelect => write!(f, "Sensors"),
            Errors => write!(f, "errors"),
            SensorSelection(Some(sensor)) => {
                write!(
                    f,
                    "Topf:{}Feu:{}\nMess:{}({:?},{:?})",
                    sensor.get_id(),
                    sensor.get_threshold(),
                    sensor.get_moisture(),
                    sensor.get_temperature().unwrap_or(0),
                    sensor.get_humidity().unwrap_or(0)
                )
            }
            SensorSelection(None)
            | SensorSettingThreshold(None)
            | SensorSettingSelectionThreshold(None)
            | SensorSettingWateringTime(None)
            | SensorSettingSelectionWateringTime(None)
            | SensorSettingBackoffTime(None)
            | SensorSettingSelectionBackoffTime(None) => {
                write!(f, "missing\ngo back")
            }
            SensorSettingThreshold(Some(sensor)) => {
                write!(f, "Feuchte:{}\nEditable", sensor.get_threshold())
            }
            SensorSettingSelectionThreshold(Some(sensor)) => {
                write!(f, "Feuchte:{}\nAuswahl", sensor.get_threshold())
            }
            SensorSettingWateringTime(Some(sensor)) => {
                write!(f, "Bewaes:{}s\nEditable", sensor.get_watering_time())
            }
            SensorSettingSelectionWateringTime(Some(sensor)) => {
                write!(f, "Bewaes:{}s\nAuswahl", sensor.get_watering_time())
            }
            SensorSettingBackoffTime(Some(sensor)) => {
                write!(f, "Warte:{}min\nEditable", sensor.get_backoff_time())
            }
            SensorSettingSelectionBackoffTime(Some(sensor)) => {
                write!(f, "Warte:{}min\nAuswahl", sensor.get_backoff_time())
            }
        }
    }
}
