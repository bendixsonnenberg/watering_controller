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
    SensorSelection(T),
    SensorSettings(T),
}

pub trait Sensor<B> {
    fn first(builder: B) -> Self;
    fn next(self, builder: B) -> Self;
    fn prev(self, builder: B) -> Self;
    async fn get_setting(&self) -> u16;
    fn get_id(&self) -> u8;
    async fn increase_setting(self, builder: B) -> Self;
    async fn decrease_setting(self, builder: B) -> Self;
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
    pub async fn input(&mut self, input: Input) {
        use Input::*;
        use State::*;
        self.state = match (self.state.clone(), input) {
            (Errors, Left) => SensorsSelect,
            (Errors, _) => Errors,
            (SensorsSelect, Right) => Errors,
            (SensorsSelect, Enter) => SensorSelection(Sensor::first(self.builder.clone())),
            (SensorsSelect, _) => SensorsSelect,
            (SensorSelection(sensor), Enter) => SensorSettings(sensor),
            (SensorSelection(sensor), Right) => SensorSelection(sensor.next(self.builder.clone())),
            (SensorSelection(sensor), Left) => SensorSelection(sensor.prev(self.builder.clone())),
            (SensorSettings(sensor), Left) => {
                SensorSettings(sensor.increase_setting(self.builder.clone()).await)
            }
            (SensorSettings(sensor), Right) => {
                SensorSettings(sensor.decrease_setting(self.builder.clone()).await)
            }
            (SensorSelection(_), Back) => SensorsSelect,
            (SensorSettings(sensor), Back) => SensorSelection(sensor),
            (SensorSettings(sensor), Enter) => SensorSettings(sensor),
        }
    }
}
impl<T: Sensor<B> + Clone, B> Display for MenuRunner<T, B> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.state.clone() {
            State::SensorsSelect => write!(f, "Sensors"),
            State::Errors => write!(f, "errors"),
            State::SensorSelection(sensor) => {
                write!(f, "Sen:{}Thr:{}", sensor.get_id(), sensor.get_setting())
            }
            State::SensorSettings(sensor) => write!(f, "Spt:{}", sensor.get_setting()),
        }
    }
}
