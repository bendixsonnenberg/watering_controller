use std::io;

use menu::*;

// fake sensor for which every one exists
#[derive(Clone, Copy)]
struct FakeSensor {
    id: u8,
    moisture: u16,
}
impl Sensor for FakeSensor {
    fn first() -> Self {
        Self {
            id: 1,
            moisture: 1000,
        }
    }
    fn next(self) -> Self {
        let mut a = self;
        a.id = self.id.wrapping_add(1);
        a
    }
    fn prev(self) -> Self {
        let mut a = self;
        a.id = self.id.wrapping_sub(1);
        a
    }
    fn get_setting(self) -> u16 {
        self.moisture
    }
    fn get_id(self) -> u8 {
        self.id
    }
    fn increase_setting(mut self) -> Self {
        self.moisture = self.moisture.saturating_add(1);
        self
    }
    fn decrease_setting(mut self) -> Self {
        self.moisture = self.moisture.saturating_sub(1);
        self
    }
}
fn main() {
    let mut menu = MenuRunner::<FakeSensor>::new();
    println!("{}", menu);
    let stdin = io::stdin();
    for line in stdin.lines() {
        let mut input = line.unwrap();
        input.make_ascii_lowercase();
        let command = match input.as_str() {
            "up" => Input::Up,
            "right" => Input::Right,
            "down" => Input::Down,
            "left" => Input::Left,
            "enter" => Input::Enter,
            _ => Input::Back,
        };
        menu.input(command);
        println!("{}", menu);
    }
    println!("hello");
}
