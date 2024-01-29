#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_futures::select::Either;
use embassy_stm32::exti::AnyChannel;
use embassy_stm32::exti::Channel;
use embassy_stm32::gpio::AnyPin;
use embassy_stm32::gpio::Pin;
use embassy_stm32::{
    exti::ExtiInput,
    gpio::{Input, Level, Output, Pull, Speed},
};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

enum LineCondition {
    Idle,
    Busy,
    Collision,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut idle_led = Output::new(p.PB15, Level::Low, Speed::Low);
    let mut busy_led = Output::new(p.PB14, Level::Low, Speed::Low);
    let mut collision_led = Output::new(p.PB13, Level::Low, Speed::Low);

    let rx_pin = Input::new(p.PC8, Pull::None);
    let mut rx_pin = ExtiInput::new(rx_pin, p.EXTI8);

    let mut onboard_led = Output::new(p.PA5, Level::Low, Speed::Low);

    let button = Input::new(p.PC13, Pull::None);
    let mut button = ExtiInput::new(button, p.EXTI13);

    let mut line_state = LineCondition::Idle;

    loop {
        button.wait_for_rising_edge().await;
        match line_state {
            LineCondition::Idle => {
                line_state = LineCondition::Busy;
                idle_led.set_high();
                busy_led.set_low();
                collision_led.set_low();
            },
            LineCondition::Busy => {
                line_state = LineCondition::Collision;
                idle_led.set_low();
                busy_led.set_high();
                collision_led.set_low();
            },
            LineCondition::Collision => {
                line_state = LineCondition::Idle;
                idle_led.set_low();
                busy_led.set_low();
                collision_led.set_high();
            },
        }
    }
}
