#![no_std]
#![no_main]
use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_stm32::{
    exti::ExtiInput,
    gpio::{Input, Level, Output, Pull, Speed},
};
use embassy_time::{Duration, Instant, Timer, WithTimeout};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(embassy_stm32::Config::default());
    let mut send_pin = Output::new(p.PB12, Level::Low, Speed::VeryHigh);
    let mut recieve_pin = ExtiInput::new(p.PB13, p.EXTI13, Pull::Down);

    loop {
        let mut sum: Duration = Duration::from_secs(0);
        let mut measures = 0;
        for _ in 0..10 {
            // doing 10 repetitions
            // ^
            send_pin.set_high();
            Timer::after_micros(10).await;
            send_pin.set_low();
            let Ok(_) = recieve_pin
                .wait_for_high()
                .with_timeout(Duration::from_millis(100))
                .await
            else {
                error!("ultrasonic timed out");
                continue;
            };
            let start = Instant::now();
            recieve_pin.wait_for_low().await;
            let duration = start.elapsed();
            if duration.as_micros() > 40_000 {
                error!("too large distance");
                continue;
            }
            sum += duration;
            measures += 1;
            Timer::after_millis(100).await;
        }
        if (measures == 0) {
            continue;
        }
        info!(
            "measures: {},duration: {}, distance: {} mm",
            measures,
            sum.as_micros() / measures,
            (sum.as_micros() / measures) as f32 * 0.343 / 2.0
        );
        Timer::after_secs(1).await;
    }
}
