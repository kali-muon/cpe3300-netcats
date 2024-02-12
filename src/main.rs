#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod receive;
mod transmit;

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    dma::NoDma,
    exti::ExtiInput,
    gpio::{Input, Level, Output, OutputOpenDrain, Pull, Speed},
    peripherals,
    peripherals::PC13,
    usart,
    usart::{Config as UsartConfig, Uart, UartRx},
    Config,
};
use embassy_sync::{
    blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex},
    pipe::{Pipe, Writer},
    signal::Signal,
};
use embassy_time::{Duration, Timer};
use embedded_io_async::Write;
use panic_probe as _;
use static_cell::StaticCell;

use crate::{receive::receive::receive_logic, transmit::transmit::collision_handling_tx};

#[derive(PartialEq, Copy, Clone, Format)]
enum LineCondition {
    Idle,
    Busy,
    Collision,
}

// tolerance is in percent
const PERIOD_TOLERANCE: f64 = 0.0132;
// all times here are in microseconds
const BIT_PERIOD_US: u64 = 1000;
const BIT_PERIOD: Duration = Duration::from_micros(BIT_PERIOD_US);
const BIT_PERIOD_UPPER_TOLERANCE: Duration = Duration::from_micros(
    BIT_PERIOD.as_micros() + ((BIT_PERIOD.as_micros() as f64 * PERIOD_TOLERANCE) as u64),
);
const BIT_PERIOD_LOWER_TOLERANCE: Duration = Duration::from_micros(
    BIT_PERIOD.as_micros() - ((BIT_PERIOD.as_micros() as f64 * PERIOD_TOLERANCE) as u64),
);
const HALF_BIT_PERIOD: Duration = Duration::from_micros(BIT_PERIOD.as_micros() / 2);
const HALF_BIT_PERIOD_UPPER_TOLERANCE: Duration = Duration::from_micros(
    HALF_BIT_PERIOD.as_micros() + ((HALF_BIT_PERIOD.as_micros() as f64 * PERIOD_TOLERANCE) as u64),
);
const HALF_BIT_PERIOD_LOWER_TOLERANCE: Duration = Duration::from_micros(
    HALF_BIT_PERIOD.as_micros() - ((HALF_BIT_PERIOD.as_micros() as f64 * PERIOD_TOLERANCE) as u64),
);
const IDLE_CUTOFF: Duration = Duration::from_micros(1150);
const COLLISION_CUTOFF: Duration = Duration::from_micros(1110);

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
    // STM32F446RE clock config (180 MHz)
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

    // // STM32F411RE clock config (100 MHz)
    // {
    //     use embassy_stm32::rcc::*;
    //     config.rcc.pll_src = PllSource::HSI;
    //     config.rcc.pll = Some(Pll {
    //         prediv: PllPreDiv::DIV16,
    //         mul: PllMul::MUL400,
    //         divp: Some(PllPDiv::DIV4),
    //         divq: Some(PllQDiv::DIV4),
    //         divr: Some(PllRDiv::DIV4),
    //     });
    //     config.rcc.sys = Sysclk::PLL1_P;
    //     config.rcc.ahb_pre = AHBPrescaler::DIV1;
    //     config.rcc.apb1_pre = APBPrescaler::DIV2;
    //     config.rcc.apb2_pre = APBPrescaler::DIV1;
    // }

    let p = embassy_stm32::init(config); // this does high clock speed
                                         // let p = embassy_stm32::init(Default::default()); // this does low clock speed
    info!("initialized clocks successfully!");
    let mut _onboard_led = Output::new(p.PA5, Level::Low, Speed::Low);

    let mut idle_led: Output<'static, embassy_stm32::gpio::AnyPin> =
        Output::new(p.PB15, Level::Low, Speed::High).degrade();
    let mut busy_led: Output<'static, embassy_stm32::gpio::AnyPin> =
        Output::new(p.PB14, Level::Low, Speed::High).degrade();
    let mut collision_led: Output<'static, embassy_stm32::gpio::AnyPin> =
        Output::new(p.PB13, Level::Low, Speed::High).degrade();

    // let mut tx_pin = OutputOpenDrain::new(p.PC6, Level::High, Speed::High, Pull::Down).degrade();
    // let mut tx_pin = Output::new(p.PB9, Level::High, Speed::High).degrade();
    let mut tx_pin: Output<'_, embassy_stm32::gpio::AnyPin> =
        OutputOpenDrain::new(p.PB9, Level::High, Speed::High, Pull::None).degrade(); // working open drain

    // let rx_pin = Input::new(p.PC8, Pull::Up);
    // let mut rx_pin = ExtiInput::new(rx_pin, p.EXTI8);
    let rx_pin: Input<'static, peripherals::PB8> = Input::new(p.PB8, Pull::Up);
    let mut rx_pin: ExtiInput<'static, peripherals::PB8> = ExtiInput::new(rx_pin, p.EXTI8);

    let push_button = Input::new(p.PC13, Pull::None);
    let mut push_button = ExtiInput::new(push_button, p.EXTI13);

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
    let (_tx, rx) = uart.split();

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
    spawner
        .spawn(receive_logic(
            &mut idle_led,
            &mut busy_led,
            &mut collision_led,
            &mut rx_pin,
        ))
        .unwrap();
    loop {
        Timer::after_secs(10);
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
            pipe_writer.write_all(transmission).await.unwrap();
            info!("writing: enter pushed");
            index = 0;
        }
    }
}
