#![no_std]
use byteorder::ByteOrder;
use embedded_can::Id;
use embedded_can::*;
use strum::FromRepr;
/// most commands can be used with a remote frame to get the value, and with a data frame to set it
#[derive(FromRepr, Debug, Eq, PartialEq, Copy, Clone)]
#[repr(u8)]
pub enum Commands {
    Threshold = 0,
    Hysterese = 1,
    // can't set moisture
    Moisture = 2,
}

static COMMAND_MASK: u16 = 0b000_1111_1111;
pub static MASK: StandardId = StandardId::new(COMMAND_MASK).unwrap();
pub fn get_filter_from_id(id: u8) -> StandardId {
    StandardId::new(id as u16).expect("u8 cant be larger than 11 bit number")
}
pub fn id_from_command_and_dev_id(command: Commands, id: u8) -> Option<StandardId> {
    StandardId::new(((command as u16) << 8) | id as u16)
}
/// extracts command, device_id of source and data from frame
/// if not a standard frame, return None
/// if the frame is not a data frame, the option<u16> will be none
pub fn frame_to_command_data<U: Frame>(frame: U) -> Option<(Commands, Option<u8>, Option<u16>)> {
    let Id::Standard(id) = frame.id() else {
        return None;
    };

    let command = Commands::from_repr((id.as_raw().checked_shr(8)?) as u8)?;
    let (data, dev_id): (Option<u16>, Option<u8>) = if frame.is_data_frame() && frame.dlc() >= 3 {
        (
            Some(byteorder::LittleEndian::read_u16(frame.data())),
            Some(frame.data()[2]),
        )
    } else {
        (None, None)
    };
    Some((command, dev_id, data))
}
pub fn create_data_buf(data: u16, dev_id: u8) -> [u8; 3] {
    let mut buf = [0_u8; 3];
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
