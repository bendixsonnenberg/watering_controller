#![no_std]

use core::fmt::Display;

/// this module creates a menu that can tbe interacted with by using 1 rotary encoder and 2 buttons
/// it requires a function to print to a screen and functions to handle sending values to sensors
/// this will not be an arbitrary menu, rather it will be
#[derive(Clone, Copy, Debug)]
pub enum Input {
    Enter,
    Back,
    Up,
    Right,
    Down,
    Left,
}
/// this enum contains the possible states that the menu can be in. Some states can have values,
/// and some State, Input tupels result in sideeffect, therefore this is not a true finite
/// deterministic state automata
#[derive(Clone, Copy)]
pub enum State<T: Sensor> {
    SensorsSelect,
    Errors,
    SensorSettings(T),
    SensorStats(T),
}

pub trait Sensor {
    fn first() -> Self;
    fn next(self) -> Self;
    fn prev(self) -> Self;
    fn get_setting(self) -> u16;
    fn get_id(self) -> u8;
    fn increase_setting(self) -> Self;
    fn decrease_setting(self) -> Self;
}

pub struct MenuRunner<T: Sensor + Copy + Clone> {
    state: State<T>,
}

impl<T: Sensor + Copy + Clone> Default for MenuRunner<T> {
    fn default() -> Self {
        Self::new()
    }
}
impl<T: Sensor + Copy + Clone> MenuRunner<T> {
    pub fn new() -> Self {
        Self {
            state: State::<T>::SensorsSelect,
        }
    }
    pub fn input(&mut self, input: Input) {
        use Input::*;
        use State::*;
        self.state = match (self.state, input) {
            (Errors, Left) => SensorsSelect,
            (Errors, _) => Errors,
            (SensorsSelect, Right) => Errors,
            (SensorsSelect, Enter) => SensorSettings(Sensor::first()),
            (SensorsSelect, _) => SensorsSelect,
            (SensorSettings(sensor), Enter) => SensorStats(sensor),
            (SensorSettings(sensor), Right) => SensorSettings(sensor.next()),
            (SensorSettings(sensor), Left) => SensorSettings(sensor.prev()),
            (SensorSettings(sensor), Up) => SensorSettings(sensor.increase_setting()),
            (SensorSettings(sensor), Down) => SensorSettings(sensor.decrease_setting()),
            (SensorSettings(_), Back) => SensorsSelect,
            (SensorStats(sensor), Back) => SensorSettings(sensor),
            (SensorStats(sensor), _) => SensorStats(sensor),
        }
    }
}
impl<T: Sensor + Copy + Clone> Display for MenuRunner<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.state {
            State::SensorsSelect => write!(f, "Sensors"),
            State::Errors => write!(f, "errors"),
            State::SensorSettings(sensor) => write!(
                f,
                "Sensor {}, Threshold: {}",
                sensor.get_id(),
                sensor.get_setting()
            ),
            State::SensorStats(sensor) => write!(f, "Stats for sensor: {}", sensor.get_id()),
        }
    }
}
