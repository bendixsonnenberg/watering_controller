use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use heapless::Vec;
const ERROR_STACK_SIZE: usize = 32;
/// enum that contains all errors that we can report
pub enum Error {
    Booted,
    PumpNotWorking,
    WaterReserveEmpty,
    SoilMoistureNotIncreasing,
    SensorNotCommunicating(u8),
    MoistureLevelUnrealistic(u8),
    NoInternet,
    SubFreezing(u8),
    SubFreezingTimeout,
    TempSensorFail,
}

static ERRORS: Mutex<CriticalSectionRawMutex, Vec<u16, ERROR_STACK_SIZE>> = Mutex::new(Vec::new());

fn error_to_code(e: Error) -> u16 {
    use Error::*;
    match e {
        Booted => 0,
        PumpNotWorking => 1,
        WaterReserveEmpty => 0x10,
        SoilMoistureNotIncreasing => 0x11,
        SensorNotCommunicating(sensor_id) => 0x0100 | (sensor_id as u16),
        MoistureLevelUnrealistic(sensor_id) => 0x0200 | (sensor_id as u16),
        NoInternet => 0x404,
        SubFreezing(sensor_id) => 0x600 | (sensor_id as u16),
        SubFreezingTimeout => 0x601,
        TempSensorFail => 0x699,
        
    }
}

/// adds error to the end of the stack
pub fn report_error(e: Error) {
    //TODO better error handeling
    if let Ok(mut errors) =  ERRORS.try_lock() {
        
        let _ = errors.push(error_to_code(e));
    }


}
/// returns a clone of the error stack
pub async fn get_errors() ->  Vec<u16, ERROR_STACK_SIZE>{
    ERRORS.lock().await.clone()
}
/// removes the last added error
pub async fn ack_error() {
    ERRORS.lock().await.pop();
}
