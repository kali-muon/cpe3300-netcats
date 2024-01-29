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

const BIT_PERIOD: u64 = 1000;
const BIT_PERIOD_WITH_ERROR: u64 = 1130;
const HALF_BIT_PERIOD: u64 = 500;
const HALF_BIT_PERIOD_WITH_ERROR: u64 = 520;


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
    let mut last_line_level = rx_pin.get_level();

    info!("Waiting to enter loop...");
    rx_pin.wait_for_any_edge().await;
        println!("entering loop!");
    loop {
        match rx_pin.get_level() {
            Level::High=> {
                match select(Timer::after_micros(BIT_PERIOD_WITH_ERROR), rx_pin.wait_for_falling_edge()).await {
                    Either::First(_) => {
                        // line has gone idle
                        println!("IDLE");
                        line_state = LineCondition::Idle;
                        idle_led.set_high();
                        busy_led.set_low();
                        collision_led.set_low();
                        // wait for the line to fall before continuing
                        rx_pin.wait_for_falling_edge().await;
                    },
                    Either::Second(_) => {
                        // got a falling edge, so we're busy again
                        continue;
                        // line_state = LineCondition::Busy; // theoretically, i could replace this with a continue, since the next loop sets this to busy
                    }
                }
            },
            Level::Low => {
                println!("BUSY");
                line_state = LineCondition::Busy;
                idle_led.set_low();
                busy_led.set_high();
                collision_led.set_low();
                match select(Timer::after_micros(BIT_PERIOD_WITH_ERROR), rx_pin.wait_for_rising_edge()).await {
                    Either::First(_) => {
                        // the collision timeout triggered
                        println!("COLLISION");
                        line_state = LineCondition::Collision;
                        idle_led.set_low();
                        busy_led.set_low();
                        collision_led.set_high();
                        // wait for the line to exit the collision state
                        rx_pin.wait_for_rising_edge().await; // this will need a refactor to support the random backoffs later
                    },
                    Either::Second(_) => {
                        // keep line marked as busy
                        continue;
                        // line_state = LineCondition::Busy;
                    }
                }
            }
        }
    }
}
