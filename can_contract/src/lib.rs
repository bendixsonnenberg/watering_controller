#![no_std]

use bitflags::{bitflags, bitflags_match};
use byteorder::ByteOrder;
use embedded_can::Id;
use embedded_can::*;
use strum::FromRepr;
/// most commands can be used with a remote frame to get the value, and with a data frame to set it
/// there can be at most 7 commands with standard can
#[derive(FromRepr, Debug, Eq, PartialEq, Copy, Clone)]
#[repr(u8)]
enum Commands {
    // the time the controller waters the plants in seconds
    // minimum time to wait between waterings in minutes
    Settings = 0,
    // status led, u16 data contains color value
    Light = 1,
    // can't set moisture
    Sensors = 2,
    // new device adding itself, the controller is started, every id is tested, if new sensor is
    // added announce itself to the world. If a new device is added and its id conflicts TODO
    Announce = 5,
}

#[derive(Debug, PartialEq, Copy, Clone)]
/// settings: threshold, backoff_time, watering_time
/// light: red, green, blue
/// Sensors: moisture, temp(1/10 C), humidity(promille)
/// announce(src_id, flags)
pub enum CommandData {
    /// settings for threshold, backoff_time, watering_time
    Settings(u16, u16, u16),
    // status led, u16 data contains color value
    Light(u16, u16, u16),
    LightRandom,
    LightOff,
    // can't set moisture
    // contains moisture, temperature, humidity
    // moisture will be raw sensor reading, temperature tenths of degree C, humidity tenths of percent
    Sensors(Option<u16>, Option<i16>, Option<u16>),
    // new device adding itself, the controller is started, every id is tested, if new sensor is
    // added announce itself to the world. If a new device is added and its id conflicts TODO
    // contains The sensor flags, so that when the sensor is found, we can generate a list of all its capabilities
    Announce(u8, SensorFlags),
}
bitflags! {
    #[derive(Debug, PartialEq, Copy, Clone)]
    pub struct SensorFlags:u8 {
        const SOIL_MOISTURE = 0b0000_0001;
        const AIR_TEMP = 0b0000_0010;
        const AIR_HUMIDITY = 0b0000_0100;
    }
    #[derive(Debug, PartialEq, Copy, Clone)]
    pub struct LightFlags:u8 {
        const LIGHT_RANDOM = 0b0000_0001;
        const LIGHT_OFF = 0b0000_0010;
        const LIGHT_RGB = 0b0000_0100;
    }

}
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum CommandDataContainer {
    /// All information needed for generating an Id and the data buf
    Data {
        target_id: DevId,
        src_id: DevId,
        data: CommandData,
    },
    /// the data in data will be ignored;
    Remote { target_id: DevId, data: CommandData },
}

pub type DevId = u8;
pub type CanData = u16;

static COMMAND_MASK: u16 = 0b000_1111_1111;
pub static MASK: StandardId = StandardId::new(COMMAND_MASK).unwrap();
pub const DLC: usize = 8;
pub fn get_filter_from_id(id: DevId) -> StandardId {
    StandardId::new(id as u16).expect("u8 cant be larger than 11 bit number")
}

/// extracts command, device_id of source and data from frame
/// if not a standard frame, return None
/// if the frame is not a data frame or a remote frame, the option<CanData> will be none
/// if the frame was a remote frame, the content of the can data will be 0.
/// since that can also occur if the actual data was 0; do not use this as a test
pub fn frame_to_command_data<U: Frame>(frame: U) -> Option<CommandDataContainer> {
    let Id::Standard(id) = frame.id() else {
        return None;
    };

    let command = Commands::from_repr((id.as_raw().checked_shr(8)?) as u8)?;
    let target_id = (id.as_raw()) as u8; // get the lower byte of id; that is the target id
    use Commands::*;
    // for each command there is a custom implementation of how the command data maps to the 8 possible bytes allowed by CAN
    let (src_id, data) = match command {
        Settings => {
            let buf = frame.data();
            let data_and_id = if !frame.is_data_frame() || buf.len() != DLC {
                (None, CommandData::Settings(0, 0, 0))
            } else {
                let threshold = byteorder::LE::read_u16(&buf[0..2]);
                let backoff_time = byteorder::LE::read_u16(&buf[2..4]);
                let watering_time = byteorder::LE::read_u16(&buf[4..6]);
                (
                    Some(buf[DLC - 1]),
                    CommandData::Settings(threshold, backoff_time, watering_time),
                )
            };
            data_and_id
        }
        Sensors => {
            let buf = frame.data();
            let data_and_id = if !frame.is_data_frame() || buf.len() != DLC {
                (None, CommandData::Sensors(None, None, None))
            } else {
                if let Some(sensors) = SensorFlags::from_bits(buf[6]) {
                    let moisture = if sensors.contains(SensorFlags::SOIL_MOISTURE) {
                        Some(byteorder::LE::read_u16(&buf[0..2]))
                    } else {
                        None
                    };
                    let temperature = if sensors.contains(SensorFlags::AIR_TEMP) {
                        Some(byteorder::LittleEndian::read_i16(&buf[2..4]))
                    } else {
                        None
                    };
                    let humidity = if sensors.contains(SensorFlags::AIR_HUMIDITY) {
                        Some(byteorder::LittleEndian::read_u16(&buf[4..6]))
                    } else {
                        None
                    };
                    (
                        Some(buf[DLC - 1]),
                        CommandData::Sensors(moisture, temperature, humidity),
                    )
                } else {
                    (Some(buf[DLC - 1]), CommandData::Sensors(None, None, None))
                }
            };
            data_and_id
        }
        Announce => {
            let buf = frame.data();
            let data_and_id = if !frame.is_data_frame() || buf.len() != DLC {
                (None, CommandData::Announce(0, SensorFlags::empty()))
            } else {
                (
                    Some(buf[DLC - 1]),
                    CommandData::Announce(
                        buf[1],
                        SensorFlags::from_bits(buf[0]).unwrap_or(SensorFlags::empty()),
                    ),
                )
            };
            data_and_id
        }
        Light => {
            let buf = frame.data();
            let data_and_id = if !frame.is_data_frame() || buf.len() != DLC {
                (None, CommandData::LightRandom)
            } else {
                let data = if let Some(flags) = LightFlags::from_bits(buf[6]) {
                    bitflags_match!(flags, {
                        LightFlags::LIGHT_OFF => CommandData::LightOff,
                        LightFlags::LIGHT_RGB => CommandData::Light(
                            byteorder::LittleEndian::read_u16(&buf[0..2]),
                            byteorder::LittleEndian::read_u16(&buf[2..4]),
                            byteorder::LittleEndian::read_u16(&buf[4..6])),
                        LightFlags::LIGHT_RANDOM => CommandData::LightRandom,

                        _ => CommandData::LightOff
                    })
                } else {
                    CommandData::Light(0, 0, 0)
                };
                (Some(buf[DLC - 1]), data)
            };
            data_and_id
        }
    };
    if frame.is_remote_frame() || src_id.is_none() {
        Some(CommandDataContainer::Remote {
            target_id: target_id,
            data: data,
        })
    } else {
        Some(CommandDataContainer::Data {
            target_id: target_id,
            src_id: src_id.expect("checked if it is none"),
            data: data,
        })
    }
}
// if src_id is none, we are returning an empty buf since it is a remote frame
// if we change anything in here we also need to change the related code in
//frame_to_command_data
fn buf_from_command_data(data: CommandData, src_id: Option<DevId>) -> [u8; DLC] {
    // DLC is 8 for can,
    let mut buf = [0; DLC];
    if let Some(src_id) = src_id {
        match data {
            CommandData::Settings(threshold, backoff_time, watering_time) => {
                byteorder::LittleEndian::write_u16(&mut buf[0..2], threshold);
                byteorder::LittleEndian::write_u16(&mut buf[2..4], backoff_time);
                byteorder::LittleEndian::write_u16(&mut buf[4..6], watering_time);
            }
            CommandData::Light(red, green, blue) => {
                buf[6] = LightFlags::LIGHT_RGB.bits();
                byteorder::LittleEndian::write_u16(&mut buf[0..2], red);
                byteorder::LittleEndian::write_u16(&mut buf[2..4], green);
                byteorder::LittleEndian::write_u16(&mut buf[4..6], blue);
            }
            CommandData::LightRandom => {
                buf[6] = LightFlags::LIGHT_RANDOM.bits();
            }
            CommandData::LightOff => {
                buf[6] = LightFlags::LIGHT_OFF.bits();
            }
            CommandData::Sensors(moisture, temp, humidity) => {
                let mut flags = SensorFlags::empty();
                if let Some(moisture) = moisture {
                    flags.insert(SensorFlags::SOIL_MOISTURE);
                    byteorder::LittleEndian::write_u16(&mut buf[0..2], moisture);
                }
                if let Some(temp) = temp {
                    flags.insert(SensorFlags::AIR_TEMP);
                    byteorder::LittleEndian::write_i16(&mut buf[2..4], temp);
                }
                if let Some(humidity) = humidity {
                    flags.insert(SensorFlags::AIR_HUMIDITY);
                    byteorder::LittleEndian::write_u16(&mut buf[4..6], humidity);
                }
                buf[6] = flags.bits();
            }
            CommandData::Announce(src_id, flags) => {
                buf[0] = flags.bits();
                buf[1] = src_id;
            }
        };
        buf[7] = src_id;
        buf
    } else {
        [0; DLC]
    }
}
fn id_from_command_and_dev_id(command: CommandData, id: DevId) -> Option<StandardId> {
    let command = match command {
        CommandData::Settings(_, _, _) => Commands::Settings,
        CommandData::Sensors(_, _, _) => Commands::Sensors,
        CommandData::Light(_, _, _) | CommandData::LightRandom | CommandData::LightOff => {
            Commands::Light
        }
        CommandData::Announce(_, _) => Commands::Announce,
    };

    StandardId::new(((command as u16) << 8) | id as u16)
}
fn create_data_buf_and_id(frame: CommandDataContainer) -> ([u8; DLC], StandardId) {
    //TODO move buf into match for variable length buffers
    let (target_id, src_id, data) = match frame {
        CommandDataContainer::Data {
            target_id,
            src_id,
            data,
        } => (target_id, Some(src_id), data),
        CommandDataContainer::Remote { target_id, data } => (target_id, None, data),
    };

    let buf = buf_from_command_data(data, src_id);
    let id = id_from_command_and_dev_id(data, target_id)
        .expect("only allowed ids will be passed by the command enum");
    (buf, id)
}
/// creates a frame of a generic type when given a ComandDataContainer
pub fn command_data_to_frame<U: embedded_can::Frame>(frame: CommandDataContainer) -> Option<U> {
    let (buf, id) = create_data_buf_and_id(frame);
    match frame {
        CommandDataContainer::Data {
            target_id: _,
            src_id: _,
            data: _,
        } => U::new(id, &buf),
        CommandDataContainer::Remote {
            target_id: _,
            data: _,
        } => U::new_remote(id, 8),
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use embedded_can::Frame;
    use mcp2515::frame::CanFrame as mcpFrame;
    #[test]
    fn test_data_buffer() {
        let (buf, _) = create_data_buf_and_id(CommandDataContainer::Data {
            target_id: 2,
            src_id: 2,
            data: CommandData::Sensors(Some(512), Some(13), None),
        });
        assert_eq!(buf, [0, 2, 13, 0, 0, 0, 0b0000_0011, 2]);
    }
    #[test]
    fn id_generation() {
        let Some(id) = id_from_command_and_dev_id(CommandData::Settings(0, 0, 0), 4) else {
            // settings : 0, id = 4 => 0b010_0000_0100
            panic!("failed creating valid id")
        };
        assert_eq!(id.as_raw(), 0x004);
    }
    fn there_and_back(data: CommandDataContainer) {
        let (buf, id) = create_data_buf_and_id(data);

        let frame = mcpFrame::new(id, &buf).expect("should not fail for valid frames");

        let data_back =
            frame_to_command_data(frame).expect("valid frame should lead to valid data");
        assert_eq!(data, data_back);
    }
    #[test]
    fn there_and_back_execution() {
        there_and_back(CommandDataContainer::Data {
            target_id: 5,
            src_id: 0,
            data: CommandData::Settings(5, 234, 512),
        });
        there_and_back(CommandDataContainer::Data {
            target_id: 5,
            src_id: 0,
            data: CommandData::Sensors(Some(5), None, None),
        });
        there_and_back(CommandDataContainer::Data {
            target_id: 5,
            src_id: 0,
            data: CommandData::Sensors(Some(5), Some(5), Some(5)),
        });
        there_and_back(CommandDataContainer::Data {
            target_id: 5,
            src_id: 0,
            data: CommandData::Light(5, 5, 5),
        });
        there_and_back(CommandDataContainer::Data {
            target_id: 5,
            src_id: 0,
            data: CommandData::LightOff,
        });
        there_and_back(CommandDataContainer::Data {
            target_id: 5,
            src_id: 0,
            data: CommandData::LightRandom,
        });
        there_and_back(CommandDataContainer::Data {
            target_id: 5,
            src_id: 0,
            data: CommandData::Announce(8, SensorFlags::SOIL_MOISTURE),
        });
    }
}
