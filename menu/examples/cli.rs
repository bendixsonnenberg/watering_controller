use std::cell::RefCell;
use std::{io, rc::Rc};

use menu::*;

static SENSORS: [(u8, u16); 3] = [(1, 100), (4, 500), (8, 90)];
// fake sensor for which every one exists
#[derive(Clone, Debug)]
struct FakeSensor {
    id: u8,
    moisture: u16,
}

impl Sensor<Rc<RefCell<SensorBuilder>>> for FakeSensor {
    fn first(builder: Rc<RefCell<SensorBuilder>>) -> Self {
        builder.borrow_mut().sensors.first().unwrap().clone()
    }
    fn next(self, builder: Rc<RefCell<SensorBuilder>>) -> Self {
        builder.borrow_mut().next(self.id).clone()
    }
    fn prev(self, builder: Rc<RefCell<SensorBuilder>>) -> Self {
        builder.borrow_mut().prev(self.id).clone()
    }
    fn get_threshold(&self) -> u16 {
        self.moisture
    }
    fn get_id(&self) -> u8 {
        self.id
    }
    fn increase_threshold(mut self, builder: Rc<RefCell<SensorBuilder>>) -> Self {
        let pos = builder.borrow().pos(self.id);
        // update global repository
        builder.borrow_mut().sensors.get_mut(pos).unwrap().moisture += 1;
        // update display
        self.moisture += 1;
        self
    }
    fn decrease_threshold(mut self, builder: Rc<RefCell<SensorBuilder>>) -> Self {
        let pos = builder.borrow().pos(self.id);
        // update global repository
        builder.borrow_mut().sensors.get_mut(pos).unwrap().moisture -= 1;
        // update display
        self.moisture -= 1;
        self
    }
}

#[derive(Clone, Debug)]
struct SensorBuilder {
    sensors: Vec<FakeSensor>,
}
impl SensorBuilder {
    fn new_sensor(&self, id: u8, moisture: u16) -> FakeSensor {
        FakeSensor { id, moisture }
    }
    fn new() -> Self {
        Self { sensors: vec![] }
    }
    fn pos(&self, id: u8) -> usize {
        self.sensors.iter().position(|f| f.id == id).unwrap()
    }
    fn next(&self, id: u8) -> &FakeSensor {
        let pos = self.pos(id);
        self.sensors.iter().cycle().nth(pos + 1).unwrap()
    }
    fn prev(&self, id: u8) -> &FakeSensor {
        let pos = self.pos(id);
        if pos > 0 {
            self.sensors.get(pos - 1).unwrap()
        } else {
            self.sensors.last().unwrap()
        }
    }
}
fn main() {
    let mut builder = Rc::new(RefCell::new(SensorBuilder::new()));
    {
        // setting up sensors
        let mut b = builder.borrow_mut();

        let sensor = b.new_sensor(5, 1500);
        b.sensors.push(sensor);
        let sensor = b.new_sensor(8, 1800);
        b.sensors.push(sensor);
        let sensor = b.new_sensor(9, 1900);
        b.sensors.push(sensor);
    }
    let mut menu = MenuRunner::<FakeSensor, Rc<RefCell<SensorBuilder>>>::new(builder);
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
