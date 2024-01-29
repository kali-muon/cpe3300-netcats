#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{exti::ExtiInput, gpio::{Input, Level, Output, Pull, Speed}};
use embassy_time::Timer;
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

    let mut idle_led = Output::new(p.PB5, Level::Low, Speed::Low);
    let mut busy_led = Output::new(p.PB6, Level::Low, Speed::Low);
    let mut collision_led = Output::new(p.PB7, Level::Low, Speed::Low);

    let rx_thing = Input::new(p.PC8, Pull::None);
    let mut rx_pin = ExtiInput::new(rx_thing, p.EXTI8);

    let mut line_state = LineCondition::Idle;

    loop {}
}
