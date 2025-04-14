#![no_std]
use byteorder::ByteOrder;
use embedded_can::Id;
use embedded_can::*;
use strum::FromRepr;

#[derive(FromRepr, Debug)]
#[repr(u8)]
pub enum Commands {
    Threshold = 0,
    Hysterese = 1,
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
/// extracts command, device_id and data from frame
/// if not a standard frame, return None
/// if the frame is not a data frame, the option<u16> will be none
pub fn frame_to_command_data<U: Frame>(frame: U) -> Option<(Commands, u8, Option<u16>)> {
    let Id::Standard(id) = frame.id() else {
        return None;
    };

    let command = Commands::from_repr((id.as_raw().checked_shr(8)?) as u8)?;
    let dev_id = (id.as_raw() & COMMAND_MASK) as u8;
    let data: Option<u16> = if frame.is_data_frame() && frame.dlc() >= 2 {
        Some(byteorder::LittleEndian::read_u16(frame.data()))
    } else {
        None
    };
    Some((command, dev_id, data))
}
pub fn write_data_buff(data: u16, buff: &mut [u8]) -> Option<()> {
    if buff.len() < 2 {
        return None;
    }

    byteorder::LittleEndian::write_u16(buff, data);
    Some(())
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_data_buffer() {
        let mut buf: [u8; 2] = [0, 0];
        write_data_buff(512, &mut buf);
        assert_eq!(buf, [0, 2]);
    }
}
