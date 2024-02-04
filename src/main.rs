#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use cortex_m::register::basepri::read;
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_futures::select::Either;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::RingBufferedUartRx;
use embassy_stm32::usart::UartRx;
use embassy_stm32::Config;
use embassy_stm32::{
    bind_interrupts,
    usart,
    peripherals,
    exti::ExtiInput,
    gpio::{AnyPin, Input, Level, Output, Pull, Speed},
};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use embassy_stm32::dma::NoDma;
use embassy_stm32::usart::{Config as UsartConfig, Uart};

enum LineCondition {
    Idle,
    Busy,
    Collision,
}

const BIT_PERIOD: u64 = 1000;
const HALF_BIT_PERIOD: u64 = 500;
const IDLE_CUTOFF: u64 = 1150;
const COLLISION_CUTOFF: u64 = 1110;

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.pll_src = PllSource::HSI;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV8,
            mul: PllMul::MUL180,
            divp: Some(PllPDiv::DIV2),
            divq: Some(PllQDiv::DIV2),
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;
    }

    let p = embassy_stm32::init(config); // this does high clock speed
    // let p = embassy_stm32::init(Default::default()); // this does low clock speed

    let mut _onboard_led = Output::new(p.PA5, Level::Low, Speed::Low);

    let mut idle_led = Output::new(p.PB15, Level::Low, Speed::High).degrade();
    let mut busy_led = Output::new(p.PB14, Level::Low, Speed::High).degrade();
    let mut collision_led = Output::new(p.PB13, Level::Low, Speed::High).degrade();

    let rx_pin = Input::new(p.PC8, Pull::None);
    let mut rx_pin = ExtiInput::new(rx_pin, p.EXTI8);

    let mut line_state = LineCondition::Idle;


    let usart_config = UsartConfig::default();
    let mut uart = Uart::new(p.USART1, p.PA10, p.PA9, Irqs, NoDma, p.DMA2_CH2, usart_config).unwrap();
    let (_tx, mut rx) = uart.split();

    spawner.spawn(uart_task(rx)).unwrap();

    // // let mut buf = [0u8; 256];
    // let rx = rx.into_ring_buffered(&mut buf);


    //info!("entering loop!");
    loop {
        match rx_pin.get_level() {
            Level::High => {
                match select(
                    Timer::after_micros(IDLE_CUTOFF),
                    rx_pin.wait_for_falling_edge(),
                )
                .await
                {
                    Either::First(_) => {
                        // line has gone idle
                        //info!("IDLE");
                        line_state = LineCondition::Idle;
                        set_status_leds(
                            &mut idle_led,
                            &mut busy_led,
                            &mut collision_led,
                            line_state,
                        );
                        // wait for the line to fall before continuing
                        rx_pin.wait_for_falling_edge().await;
                    }
                    Either::Second(_) => {
                        // got a falling edge, so we're busy again
                        continue;
                    }
                }
            }
            Level::Low => {
                // info!("BUSY");
                line_state = LineCondition::Busy;
                set_status_leds(&mut idle_led, &mut busy_led, &mut collision_led, line_state);
                match select(
                    Timer::after_micros(COLLISION_CUTOFF),
                    rx_pin.wait_for_rising_edge(),
                )
                .await
                {
                    Either::First(_) => {
                        // the collision timeout triggered
                        //info!("COLLISION");
                        line_state = LineCondition::Collision;
                        set_status_leds(
                            &mut idle_led,
                            &mut busy_led,
                            &mut collision_led,
                            line_state,
                        );
                        // wait for the line to exit the collision state
                        rx_pin.wait_for_rising_edge().await; // this will need a refactor to support the random backoffs later
                    }
                    Either::Second(_) => {
                        // keep line marked as busy
                        continue;
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn uart_task(mut rx: UartRx<'static, peripherals::USART1, peripherals::DMA2_CH2>) {
    let mut dma_buf = [0u8; 1024];
    let mut read_buf = [0u8; 1024];
    let mut test3 = rx.into_ring_buffered(&mut dma_buf);
    //test3.start().unwrap();
    loop {
        let n = test3.read(&mut read_buf).await.unwrap();
        info!("{}", n);
    }
}

fn set_status_leds(
    idle_led: &mut Output<'_, AnyPin>,
    busy_led: &mut Output<'_, AnyPin>,
    collision_led: &mut Output<'_, AnyPin>,
    line_state: LineCondition,
) {
    match line_state {
        LineCondition::Idle => {
            idle_led.set_high();
            busy_led.set_low();
            collision_led.set_low();
        }
        LineCondition::Busy => {
            idle_led.set_low();
            busy_led.set_high();
            collision_led.set_low();
        }
        LineCondition::Collision => {
            idle_led.set_low();
            busy_led.set_low();
            collision_led.set_high();
        }
    }
}
