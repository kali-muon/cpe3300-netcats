#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{exti::ExtiInput, gpio::{Input, Level, Output, Pull, Speed}};
use embassy_time::{Timer};
use embassy_futures::select::select;
use embassy_futures::select::Either;
use {defmt_rtt as _, panic_probe as _};

enum LineCondition {
    Idle,
    Busy,
    Collision,
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut idle_led = Output::new(p.PB15, Level::Low, Speed::Low);
    let mut busy_led = Output::new(p.PB14, Level::Low, Speed::Low);
    let mut collision_led = Output::new(p.PB13, Level::Low, Speed::Low);

    let rx_pin = Input::new(p.PC8, Pull::None);
    let mut rx_pin = ExtiInput::new(rx_pin, p.EXTI8);

    let mut line_state = LineCondition::Idle;

    let mut onboard_led = Output::new(p.PA5, Level::Low, Speed::Low);

    loop {
        info!("toggling! {}", embassy_time::TICK_HZ);
        collision_led.toggle();
        Timer::after_millis(100).await;
    }
}
