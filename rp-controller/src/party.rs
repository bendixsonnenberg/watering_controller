use can_contract::{CommandData, CommandDataContainer};

use crate::{
    SensorBitmap,
    can::{CanSender, set_value},
};

pub async fn party(sensors: &SensorBitmap, mut can_tx: CanSender, enable: bool) {
    sensors.lock(|sensors| {
        for sensor in sensors.borrow().into_iter() {
            set_value(
                &mut can_tx,
                CommandDataContainer::Data {
                    target_id: sensor as u8,
                    src_id: 0,
                    data: if enable {
                        CommandData::LightRandom
                    } else {
                        CommandData::LightOff
                    },
                },
            );
        }
    })
}
