#![no_std]
use byteorder::ByteOrder;
use embedded_can::Id;
use embedded_can::*;
use strum::FromRepr;
/// most commands can be used with a remote frame to get the value, and with a data frame to set it
/// there can be at most 7 commands with standard can
#[derive(FromRepr, Debug, Eq, PartialEq, Copy, Clone)]
#[repr(u8)]
pub enum Commands {
    Threshold = 0,
    // can't set moisture
    Moisture = 2,
    // the time the controller waters the plants in seconds
    WateringTime = 3,
    // minimum time to wait between waterings in minutes
    BackoffTime = 4,
    // new device adding itself, the controller is started, every id is tested, if new sensor is
    // added announce itself to the world. If a new device is added and its id conflicts TODO
    Announce = 5,
}

pub type DevId = u8;
pub type CanData = u16;

static COMMAND_MASK: u16 = 0b000_1111_1111;
pub static MASK: StandardId = StandardId::new(COMMAND_MASK).unwrap();
pub const DLC: usize = 3;
pub fn get_filter_from_id(id: DevId) -> StandardId {
    StandardId::new(id as u16).expect("u8 cant be larger than 11 bit number")
}
pub fn id_from_command_and_dev_id(command: Commands, id: DevId) -> Option<StandardId> {
    StandardId::new(((command as u16) << 8) | id as u16)
}
/// extracts command, device_id of source and data from frame
/// if not a standard frame, return None
/// if the frame is not a data frame, the option<CanData> will be none
pub fn frame_to_command_data<U: Frame>(
    frame: U,
) -> Option<(Commands, Option<DevId>, Option<CanData>)> {
    let Id::Standard(id) = frame.id() else {
        return None;
    };

    let command = Commands::from_repr((id.as_raw().checked_shr(8)?) as u8)?;
    let (data, dev_id): (Option<CanData>, Option<DevId>) =
        if frame.is_data_frame() && frame.dlc() >= DLC {
            (
                Some(byteorder::LittleEndian::read_u16(frame.data())),
                Some(frame.data()[2]),
            )
        } else {
            (None, None)
        };
    Some((command, dev_id, data))
}
pub fn create_data_buf(data: CanData, dev_id: DevId) -> [u8; DLC] {
    let mut buf = [0_u8; DLC];
    byteorder::LittleEndian::write_u16(&mut buf, data);
    buf[2] = dev_id;
    buf
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_data_buffer() {
        let buf = create_data_buf(512, 2);
        assert_eq!(buf, [0, 2, 2]);
    }
    #[test]
    fn id_generation() {
        let Some(id) = id_from_command_and_dev_id(Commands::Moisture, 4) else {
            // moisture : 2, id = 4 => 0b010_0000_0100
            panic!("failed creating valid id")
        };
        assert_eq!(id.as_raw(), 0x204);
    }
}
