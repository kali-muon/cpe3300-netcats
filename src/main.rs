#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use bitvec::prelude::*;
use cortex_m::register::basepri::read;
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_futures::select::Either;
use embassy_stm32::gpio::OutputOpenDrain;
use embassy_stm32::peripherals::PC13;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::RingBufferedUartRx;
use embassy_stm32::usart::UartRx;
use embassy_stm32::Config;
use embassy_stm32::{
    bind_interrupts,
    exti::ExtiInput,
    gpio::{AnyPin, Input, Level, Output, Pull, Speed},
    peripherals, usart,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::pipe::{Pipe, Reader, Writer};
use embassy_sync::signal::Signal;
use embassy_time::Duration;
use embassy_time::Ticker;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use embassy_stm32::dma::NoDma;
use embassy_stm32::usart::{Config as UsartConfig, Uart};

use embedded_io_async::Write;

use static_cell::StaticCell;

#[derive(PartialEq)]
enum LineCondition {
    Idle,
    Busy,
    Collision,
}

const BIT_PERIOD: u64 = 1000;
const HALF_BIT_PERIOD: u64 = BIT_PERIOD / 2;
const IDLE_CUTOFF: u64 = 1150;
const COLLISION_CUTOFF: u64 = 1110;

const NULLS_COMMAND: &[u8] = b"\\nulls";
const NULLS_PAYLOAD: &[u8] = b"\0\0\0\0\0\0\0\0";
const AA_COMMAND: &[u8] = b"\\aa";
const AA_PAYLOAD: &[u8] = b"\xaa";
const LONG_COMMAND: &[u8] = b"\\long";
const LONG_PAYLOAD: &[u8] = b"dsfkghfdlgdjfgo;irxgdjomgixrdlkfdfjdrlmgidfjggkjxfvxilrdjgxldfigjdnrgfldgkmxdivjfligjcfmkdj.stp9ergsgleirhg438r7tous85etj4w98r23uorjew8rm43fj488nto7ewmtw357i4omtw5ex9xwu5o8txj3o874i273z6tvy235rv247rc48xri39t,reim4ctx,xjowirjxt498txexworixremtojisdkkk";

static STATE_SIGNAL: Signal<ThreadModeRawMutex, LineCondition> = Signal::new();

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
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

    // let mut tx_pin = OutputOpenDrain::new(p.PC6, Level::High, Speed::High, Pull::Down).degrade();
    // let rx_pin = Input::new(p.PC8, Pull::Up);
    // let mut rx_pin = ExtiInput::new(rx_pin, p.EXTI8);
    // let mut tx_pin = OutputOpenDrain::new(p.PB9, Level::High, Speed::High, Pull::Down).degrade();
    let mut tx_pin = Output::new(p.PB9, Level::High, Speed::High).degrade();
    let rx_pin = Input::new(p.PB8, Pull::Up);
    let mut rx_pin = ExtiInput::new(rx_pin, p.EXTI8);

    let push_button = Input::new(p.PC13, Pull::None);
    let mut push_button = ExtiInput::new(push_button, p.EXTI13);

    let mut line_state = LineCondition::Idle;

    let usart_config = UsartConfig::default();
    let mut uart = Uart::new(
        p.USART1,
        p.PA10,
        p.PA9,
        Irqs,
        NoDma,
        p.DMA2_CH2,
        usart_config,
    )
    .unwrap();
    let (_tx, mut rx) = uart.split();

    let pipe: &'static mut Pipe<NoopRawMutex, 256> =
        STATIC_PIPE.init(Pipe::<NoopRawMutex, 256>::new());
    let (reader, writer) = pipe.split();

    spawner.spawn(uart_task(rx, writer.clone())).unwrap();
    spawner
        .spawn(collision_handling_tx(tx_pin, reader))
        .unwrap();
    spawner
        .spawn(button_tx(push_button, writer.clone()))
        .unwrap();

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
                        STATE_SIGNAL.signal(LineCondition::Idle);
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
                STATE_SIGNAL.signal(LineCondition::Busy);
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
                        STATE_SIGNAL.signal(LineCondition::Collision);
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
async fn button_tx(
    mut button: ExtiInput<'static, PC13>,
    mut writer: Writer<'static, NoopRawMutex, 256>,
) {
    loop {
        button.wait_for_rising_edge().await;
        writer.write_all(b"Hello World").await.unwrap();
    }
}

static STATIC_PIPE: StaticCell<Pipe<NoopRawMutex, 256>> = StaticCell::new();

#[embassy_executor::task]
async fn collision_handling_tx(
    mut tx_pin: Output<'static, AnyPin>,
    reader: Reader<'static, NoopRawMutex, 256>,
) {
    info!("starting collision handling tx");
    let mut tx_buf = [0u8; 256];
    let mut tx_ticker = Ticker::every(Duration::from_micros(HALF_BIT_PERIOD));

    loop {
        let n = reader.read(&mut tx_buf).await;
        info!("n = {}",n);
        // let word = core::str::from_utf8(&tx_buf[..n]).unwrap();
        let tx_bits: &BitSlice<u8, Msb0> = BitSlice::from_slice(&tx_buf[..n]);
        info!("received text");
        loop {
            info!("transmitting and waiting");
            line_condition_wait_until(LineCondition::Idle).await;
            match select(
                manchester_tx(&mut tx_pin, &mut tx_ticker, &tx_bits),
                line_condition_wait_until(LineCondition::Collision),
            )
            .await
            {
                Either::First(_) => {
                    info!("finished transmititng");
                    break;
                } // finished
                Either::Second(_) => {
                    tx_pin.set_high();
                    // back off
                    //continue;
                    info!("Stopped transmitting due to collision!");
                    break;
                }
            }
        }
        // info!("word length {}: {}\nrecv length: {}", word.len(), word, n);
    }
}

async fn line_condition_wait_until(condition: LineCondition) {
    info!("entering wait_until");
    while STATE_SIGNAL.wait().await != condition {
        STATE_SIGNAL.reset();
    } // exits when signal == condition
    STATE_SIGNAL.reset();
    info!("exiting wait_until");
}

async fn manchester_tx(
    tx_pin: &mut Output<'_, AnyPin>,
    ticker: &mut Ticker,
    tx_bits: &BitSlice<u8, Msb0>,
) {
    info!("starting tx");
    ticker.reset(); // prepare the ticker for the transmit
    for b in tx_bits.iter().by_vals() {
        // check state first!!
        match b {
            true => {
                // transmit a 1
                tx_pin.set_low();
                // Timer::after_micros(HALF_BIT_PERIOD).await;
                ticker.next().await;
                tx_pin.set_high();
                // Timer::after_micros(HALF_BIT_PERIOD).await;
                ticker.next().await;
            }
            false => {
                // transmit a 0
                tx_pin.set_high();
                // Timer::after_micros(HALF_BIT_PERIOD).await;
                ticker.next().await;
                tx_pin.set_low();
                // Timer::after_micros(HALF_BIT_PERIOD).await;
                ticker.next().await;
            }
        }
    }
    tx_pin.set_high();
    info!("finished tx");
}

#[embassy_executor::task]
async fn uart_task(
    rx: UartRx<'static, peripherals::USART1, peripherals::DMA2_CH2>,
    mut pipe_writer: Writer<'static, NoopRawMutex, 256>,
) {
    let mut dma_buf = [0u8; 1024];
    let mut read_buf = [0u8; 256];
    let mut sending_buf = [0u8; 256];
    let mut index = 0;
    let mut ring_uart = rx.into_ring_buffered(&mut dma_buf);
    loop {
        info!("waiting for uart");
        let number_characters_read = ring_uart.read(&mut read_buf).await.unwrap();
        info!(
            "character received\nindex = {}\nnchar = {}",
            index, number_characters_read
        );
        sending_buf[index..index + number_characters_read]
            .copy_from_slice(&read_buf[..number_characters_read]);
        index += number_characters_read;
        info!("copy from slice successful");

        //info!("sucessfully read: {}", read_buf);
        if read_buf.contains(&b'\r') {
            let message = &sending_buf[..index - 1]; // -1 to strip carriage return
            let transmission = match message {
                NULLS_COMMAND => NULLS_PAYLOAD,
                AA_COMMAND => AA_PAYLOAD,
                LONG_COMMAND => LONG_PAYLOAD,
                _ => message,
            };
            info!("writing to pipe");
            pipe_writer.write_all(transmission).await.unwrap(); 
            info!("writing: enter pushed");
            index = 0;
        }
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
