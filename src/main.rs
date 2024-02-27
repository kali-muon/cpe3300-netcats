#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use bitvec::prelude::*;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_stm32::{
    adc::{AdcPin, Resolution},
    bind_interrupts,
    dma::NoDma,
    exti::ExtiInput,
    gpio::{AnyPin, Input, Level, Output, OutputOpenDrain, OutputType, Pull, Speed},
    pac::adc::Adc,
    peripherals::{self, PC13},
    timer::simple_pwm::{PwmPin, SimplePwm},
    usart::{self, Config as UsartConfig, Uart, UartRx},
    Config,
};
use embassy_sync::{
    blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex},
    pipe::{Pipe, Reader, Writer},
    signal::Signal,
};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io_async::Write;
use panic_probe as _;
use static_cell::StaticCell;
use crc::{Crc, Algorithm, CRC_16_IBM_SDLC, CRC_32_ISCSI};

#[derive(PartialEq, Copy, Clone, Format)]
enum LineCondition {
    Idle,
    Busy,
    Collision,
}

#[derive(Clone, Copy)]
struct Edge {
    level: Level,
    time: Instant,
}

impl Edge {
    fn update(&mut self, level: Level) {
        self.level = level;
        self.time = Instant::now();
    }

    fn get_timeout_time(&self) -> Instant {
        match self.level {
            Level::High => self.time + IDLE_CUTOFF,
            Level::Low => self.time + COLLISION_CUTOFF,
        }
    }
    fn get_timeout_state(&self) -> LineCondition {
        match self.level {
            Level::High => LineCondition::Idle,
            Level::Low => LineCondition::Collision,
        }
    }
}

struct Packet {
    preamble: u8,
    source: u8,
    destination: u8,
    length: u8,
    crc_flag: bool,
    message: [u8; 255],
    trailer: u8,
}

// tx_bits: &BitSlice<u8, Msb0>,
impl Packet {
    fn to_u8_slice(&self, slice: &mut [u8]) -> usize {
        slice[0] = self.preamble;
        slice[1] = self.source;
        slice[2] = self.destination;
        slice[3] = self.length;
        slice[4] = self.crc_flag.into();
        slice[5..5 + self.length as usize].copy_from_slice(&self.message[0..self.length as usize]);
        slice[5 + self.length as usize] = self.trailer;

        self.length as usize + 6 // 6 is the total number of bytes in the header and trailer
    }
    fn to_bit_slice(&self, slice: &mut BitSlice<u8, Msb0>) -> u8  {
        // Convert each field to a bitslice
        let source_bits = BitSlice::<u8, Msb0>::from_element(&self.source);
        let destination_bits = BitSlice::<u8, Msb0>::from_element(&self.destination);
        let length_bits = BitSlice::<u8, Msb0>::from_element(&self.length);
        let crc_flag_u8: u8 = self.crc_flag.into();
        let crc_flag_bits = BitSlice::<u8, Msb0>::from_element(&crc_flag_u8);
        let trailer_bits = BitSlice::<u8, Msb0>::from_element(&self.trailer);

        // Copy each bitslice to the target slice
        slice[0..8].copy_from_bitslice(source_bits);
        slice[8..16].copy_from_bitslice(destination_bits);
        slice[16..24].copy_from_bitslice(length_bits);
        slice[24..32].copy_from_bitslice(crc_flag_bits);
        slice[32..32 + self.length as usize * 8].copy_from_bitslice(BitSlice::<u8, Msb0>::from_slice(&self.message));
        slice[32 + self.length as usize * 8..40 + self.length as usize * 8].copy_from_bitslice(&trailer_bits);
        // Asher: "Copilot wrote a great function here." (one minute later): "I'm going to throw you directly into the sea."
        // (five minutes later): "I am in Hell. I am in Hell, and there are no funny animated characters."
        // Kali: "i hate this function. i don't know if i can trust the artificial intelligence."
        // Asher: "I wholeheartedly trust the artificial intelligence. There's no way it can mess up." (foreshadowing???)
        self.length + 5 // 5 is the total number of bytes in the header and trailer
    }
    fn new(destination: u8, message: &[u8]) -> (Self, usize) {
        let mut message_holder = [0u8; 255];
        let length = message.len().min(255);
        message_holder[0..length].copy_from_slice(&message[0..length]);
        let packet = Self {
            preamble: 0x55,
            source: DEVICE_ADDRESS,
            destination,
            length: length as u8,
            crc_flag: false,
            message: message_holder,
            trailer: TRAILER_NO_CRC,
        };
        let mut packet_holder = [0u8; 262];
        let packet_length = packet.to_u8_slice(&mut packet_holder);
        (packet, packet_length)
    }
    fn explain_Alan_Wake(&self) {
        println!("Alan Wake is a character from a video game developed by Remedy Entertainment. He is a best-selling thriller novelist suffering from writer's block. He goes on a vacation to the small town of Bright Falls with his wife, Alice, to recover. However, Alice disappears under mysterious circumstances, and Wake finds himself experiencing events from his own novels.");
        println!("In the game, Wake is forced to confront the supernatural and the darkness that lurks within himself and around the town. He uses a flashlight and other light-based weapons to combat the darkness and the shadowy creatures it spawns.");
        println!("The game's story is presented in an episodic format, with elements of suspense and psychological thriller, drawing inspiration from TV shows like Twin Peaks and The Twilight Zone.");
        println!("Alan Wake's journey is not just a physical battle, but a psychological one as well, as he struggles with his own sanity, the mystery of his wife's disappearance, and the very fabric of reality itself.");
        println!("Throughout the game, Wake is guided by a mysterious voice on the radio and finds pages of a manuscript scattered around the town. These pages, which Wake doesn't remember writing, predict events that soon come to pass and provide clues to help him navigate the challenges he faces.");
        println!("The game's setting, Bright Falls, is a character in its own right. The town is filled with quirky inhabitants and is surrounded by a dense forest and a large body of water, which all add to the game's eerie atmosphere.");
        println!("The game also features a 'day-night' cycle. During the day, Wake can interact with the townsfolk and explore Bright Falls without fear. But when night falls, the town transforms into a terrifying landscape filled with shadowy figures.");
        println!("The game's narrative explores themes of light versus darkness, reality versus fiction, and the power of art and creativity. Wake's journey is a metaphor for the creative process, with his battle against the darkness mirroring the struggles that a writer faces in bringing a story to life.");
        println!("Alan Wake is not just a character, but a symbol of the struggle between light and darkness, reality and fiction, sanity and madness. His journey is a testament to the power of the human spirit and the enduring power of storytelling.");
        println!("The game's mechanics also reflect its narrative themes. The use of light to combat the darkness is not just a gameplay mechanic, but also a metaphor for Wake's struggle against the forces that threaten to consume him.");
        println!("The game's episodic structure, with each episode ending in a cliffhanger, creates a sense of tension and urgency that mirrors Wake's own feelings of confusion and fear.");
        println!("The game also features a variety of environmental puzzles that Wake must solve to progress. These puzzles often involve using light to interact with the environment in creative ways, further reinforcing the game's themes of light versus darkness and reality versus fiction.");
        println!("The game's soundtrack, composed by Petri Alanko, also plays a crucial role in setting the mood and atmosphere. The music is haunting and atmospheric, with a mix of orchestral and electronic elements that reflect the game's blend of reality and the supernatural.");
        println!("In addition to the main game, there are two special episodes titled 'The Signal' and 'The Writer'. These episodes delve deeper into Wake's psyche and provide additional context and backstory to the events of the main game.");
        println!("Alan Wake's story is also expanded in a spin-off game titled 'Alan Wake's American Nightmare'. In this game, Wake finds himself trapped in an episode of 'Night Springs', a fictional TV show within the Alan Wake universe.");
        println!("Throughout his journey, Wake encounters a variety of characters, each with their own unique personalities and roles to play in the story. These characters, along with the town of Bright Falls itself, add depth and richness to the game's narrative.");
        println!("Alan Wake's story is filled with twists and turns, keeping players on the edge of their seats. The game's narrative is masterfully crafted, with each revelation adding a new layer of complexity to the story.");
        println!("In conclusion, Alan Wake is a complex and layered character whose journey is a compelling exploration of themes like light versus darkness, reality versus fiction, and the power of art and creativity. His story is a testament to the power of the human spirit and the enduring power of storytelling.");
    }
        
}

// tolerance is in percent
const PERIOD_TOLERANCE: f64 = 0.035;
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
//const TX_HALF_BIT_PERIOD: Duration = Duration::from_micros(493);
const TX_HALF_BIT_PERIOD: Duration = HALF_BIT_PERIOD;
const IDLE_CUTOFF: Duration = Duration::from_micros(1150);
const COLLISION_CUTOFF: Duration = Duration::from_micros(1110);

const NULLS_COMMAND: &[u8] = b"\\nulls";
const NULLS_PAYLOAD: &[u8] = &[0u8; 30];
const NULLS_PRE_COMMAND: &[u8] = b"\\nullspre";
const NULLS_PRE_PAYLOAD: &[u8] = b"\x55\0\0\0\0\0\0\0\0";
const ECONOMY_COMMAND: &[u8] = b"\\economy";
const ECONOMY_PAYLOAD: &[u8] = b"Hello World. I love the economy!";
const AA_COMMAND: &[u8] = b"\\aa";
const AA_PAYLOAD: &[u8] = b"\xaa";
const POUND_COMMAND: &[u8] = b"\\pound";
const POUND_PAYLOAD: &[u8] = b"\x55\xa3";
const LONG_COMMAND: &[u8] = b"\\long";
const LONG_PAYLOAD: &[u8] = b"Udsfkghfdlgdjfgo;irxgdjomgixrdlkfdfjdrlmgidfjggkjxfvxilrdjgxldfigjdnrgfldgkmxdivjfligjcfmkdj.stp9ergsgleirhg438r7tous85etj4w98r23uorjew8rm43fj488nto7ewmtw357i4omtw5ex9xwu5o8txj3o874i273z6tvy235rv247rc48xri39t,reim4ctx,xjowirjxt498txexworixremtojisdkkk";
const FIVES_COMMAND: &[u8] = b"\\fives";
const FIVES_PAYLOAD: &[u8] = &[0x55u8; 30];
const ONES_COMMAND: &[u8] = b"\\ones";
const ONES_PAYLOAD: &[u8] = &[0xFFu8; 30];
const ALAN_COMMAND_1: &[u8] = b"\\alanwake1";
const ALAN_PAYLOAD_1: &[u8] = b"Alan Wake is a character from a video game developed by Remedy Entertainment. He is a best-selling thriller novelist suffering from writer's block. He goes on a vacation to the small town of Bright Falls with his wife, Alice, to recover.";
const ALAN_COMMAND_2: &[u8] = b"\\alanwake2";
const ALAN_PAYLOAD_2: &[u8] = b"However, Alice disappears under mysterious circumstances, and Wake finds himself experiencing events from his own novels.";

static STATE_SIGNAL: Signal<ThreadModeRawMutex, LineCondition> = Signal::new();


//Header Constants
// Define constants
const PREAMBLE: u8 = 0x55;
const DEVICE_ADDRESS: u8 = 0x24;
const BROADCAST_ADDRESS: u8 = 0xFF;
const CRC_FLAG: u8 = 0;
const TRAILER_NO_CRC: u8 = 0xAA;

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
                                         //info!("initialized clocks successfully!");
    let mut _onboard_led = Output::new(p.PA5, Level::Low, Speed::Low);

    let mut idle_led = Output::new(p.PB15, Level::Low, Speed::High).degrade();
    let mut busy_led = Output::new(p.PB14, Level::Low, Speed::High).degrade();
    let mut collision_led = Output::new(p.PB13, Level::Low, Speed::High).degrade();

    // let mut tx_pin = OutputOpenDrain::new(p.PC6, Level::High, Speed::High, Pull::Down).degrade();
    // let mut tx_pin = Output::new(p.PB9, Level::High, Speed::High).degrade();
    let mut tx_pin = OutputOpenDrain::new(p.PB9, Level::High, Speed::High, Pull::Up).degrade(); // working open drain

    // let rx_pin = Input::new(p.PC8, Pull::Up);
    // let mut rx_pin = ExtiInput::new(rx_pin, p.EXTI8);
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
    let (mut _tx, mut rx) = uart.split();

    let pipe: &'static mut Pipe<NoopRawMutex, 256> =
        STATIC_PIPE.init(Pipe::<NoopRawMutex, 256>::new());
    let (reader, writer) = pipe.split();

    let ch1 = PwmPin::new_ch1(p.PB4, OutputType::PushPull);
    let mut pwm = SimplePwm::new(
        p.TIM3,
        Some(ch1),
        None,
        None,
        None,
        embassy_stm32::time::hz(1000),
        Default::default(),
    );
    let max = pwm.get_max_duty();
    pwm.enable(embassy_stm32::timer::Channel::Ch1);
    pwm.set_duty(embassy_stm32::timer::Channel::Ch1, 0);

    spawner.spawn(uart_task(rx, writer.clone())).unwrap();
    spawner
        .spawn(collision_handling_tx(tx_pin, reader))
        .unwrap();
    spawner
        .spawn(button_tx(push_button, writer.clone()))
        .unwrap();

    //info!("entering loop!");
    let mut previous_edge: Edge = Edge {
        level: Level::Low,
        time: Instant::MIN,
    };
    let mut current_edge: Edge = Edge {
        level: Level::Low,
        time: Instant::MIN,
    };
    let mut read_bit: Level = Level::Low;
    set_status_leds(&mut idle_led, &mut busy_led, &mut collision_led, line_state);

    STATE_SIGNAL.signal(line_state);
    //info!("entering outer loop");
    loop {
        //info!("top of outer loop");
        // loops once for each packet
        let mut rx_buf = [0u8; 320];
        let rx_bits: &mut BitSlice<_, Msb0> = BitSlice::from_slice_mut(&mut rx_buf);
        let mut rx_iter = rx_bits.iter_mut();
        let mut message_complete: bool = false;
        let mut bits_read: usize = 2;
        rx_iter.next().unwrap().commit(false);
        rx_iter.next().unwrap().commit(true);

        match select(
            rx_pin.wait_for_falling_edge(),
            Timer::at(current_edge.get_timeout_time()),
        )
        .await
        {
            Either::First(_) => {
                // continue
            }
            Either::Second(_) => {
                // set to idle
                // wait for falling edge
                line_state = LineCondition::Idle;
                STATE_SIGNAL.signal(line_state);
                set_status_leds(&mut idle_led, &mut busy_led, &mut collision_led, line_state);
                //info!("waiting for message to start");
                rx_pin.wait_for_falling_edge().await;
            }
        }

        // now we are out of idle
        // "begin transmission" state
        previous_edge.update(rx_pin.get_level());
        line_state = LineCondition::Busy;
        STATE_SIGNAL.signal(line_state);
        set_status_leds(&mut idle_led, &mut busy_led, &mut collision_led, line_state);
        //info!("got the first falling edge");
        let mut synchronized: bool = false;

        while !synchronized && !message_complete {
            let timeout_time = previous_edge.get_timeout_time(); // sets the collision cutoff
            match select(rx_pin.wait_for_any_edge(), Timer::at(timeout_time)).await {
                Either::First(_) => {
                    current_edge.update(rx_pin.get_level());
                    if current_edge.time.duration_since(previous_edge.time)
                        > BIT_PERIOD_LOWER_TOLERANCE
                    {
                        // exit "begin transmission state"
                        synchronized = true;
                        //info!("synchronized");
                        // enter "decoding state"
                    } else {
                        //info!("not synchronized yet");
                        // stay in "begin transmission state"
                    }
                    line_state = LineCondition::Busy;
                    STATE_SIGNAL.signal(line_state);
                    set_status_leds(&mut idle_led, &mut busy_led, &mut collision_led, line_state);
                    previous_edge = current_edge;
                }
                Either::Second(_) => {
                    line_state = previous_edge.get_timeout_state();
                    STATE_SIGNAL.signal(line_state);
                    set_status_leds(&mut idle_led, &mut busy_led, &mut collision_led, line_state);
                    // stop reception of message
                    message_complete = true;
                    //info!(
                    //    "collision while synchronizing\ncurrent state: {}",
                    //    line_state
                    //);
                    rx_pin.wait_for_rising_edge().await;
                    previous_edge = current_edge;
                    current_edge.update(rx_pin.get_level());
                }
            };
        }
        if !message_complete {
            //info!("entering decoding loop");
        }
        while !message_complete {
            // loops while receiving a packet

            // decoding state -----------------------------------------------------
            // Wait for an edge
            //info!("waiting for next bit period");
            match select(
                rx_pin.wait_for_any_edge(),
                Timer::at(previous_edge.get_timeout_time()),
            )
            .await
            {
                Either::First(_) => {
                    // edge triggered
                    // Update the current edge instantly with the new information
                    current_edge.update(rx_pin.get_level());

                    //2T since last edge
                    if current_edge.time.duration_since(previous_edge.time)
                        > BIT_PERIOD_LOWER_TOLERANCE
                    // && current_edge.time.duration_since(previous_edge.time)
                    //     < BIT_PERIOD_UPPER_TOLERANCE
                    {
                        //Send the line value
                        rx_iter.next().unwrap().commit(current_edge.level.into());
                        bits_read += 1;
                        //info!("wrote bit: {}", current_edge.level);
                        //Cycle the edge for next decode
                        // current_edge = previous_edge;
                        previous_edge = current_edge;
                        //Loop back to beginning
                    }
                    //T since last edge (half bit period)
                    else if current_edge.time.duration_since(previous_edge.time)
                        > HALF_BIT_PERIOD_LOWER_TOLERANCE
                        && current_edge.time.duration_since(previous_edge.time)
                            < HALF_BIT_PERIOD_UPPER_TOLERANCE
                    {
                        //info!("waiting for next half bit period");
                        // previous_edge = current_edge;
                        match select(
                            // wait for next half bit period
                            rx_pin.wait_for_any_edge(),
                            Timer::at(previous_edge.get_timeout_time()),
                        )
                        .await
                        {
                            Either::First(_) => {
                                // edge triggered
                                current_edge.update(rx_pin.get_level());
                                // if current_edge.time.duration_since(previous_edge.time)
                                //     > HALF_BIT_PERIOD_LOWER_TOLERANCE
                                //     && current_edge.time.duration_since(previous_edge.time)
                                //         < HALF_BIT_PERIOD_UPPER_TOLERANCE
                                if current_edge.time.duration_since(previous_edge.time)
                                    > BIT_PERIOD_LOWER_TOLERANCE
                                    && current_edge.time.duration_since(previous_edge.time)
                                        < BIT_PERIOD_UPPER_TOLERANCE
                                {
                                    //We're safe, received half a bit period

                                    //Send the line value
                                    rx_iter.next().unwrap().commit(current_edge.level.into());
                                    bits_read += 1;
                                    //info!("wrote bit: {}", current_edge.level);
                                    //Cycle the edge for next decode
                                    previous_edge = current_edge;
                                    //Loop back to beginning
                                } else {
                                    //info!("received a full bit period after a half");
                                    // We received a full bit period, so invalid data, stop reception
                                    message_complete = true;
                                }
                            }

                            Either::Second(_) => {
                                // got a collision or idle
                                line_state = previous_edge.get_timeout_state();
                                set_status_leds(
                                    &mut idle_led,
                                    &mut busy_led,
                                    &mut collision_led,
                                    line_state,
                                );
                                STATE_SIGNAL.signal(line_state);
                                //info!("timeout trying to switch value: {}", line_state);
                                //Stop receiving the transmission
                                message_complete = true;
                            }
                        }
                    }
                    //Non-valid time since last edge
                    else {
                        //info!("invalid time since last edge");
                        previous_edge = current_edge;
                        //Invalid data, stop reception
                    }
                }

                Either::Second(_) => {
                    line_state = previous_edge.get_timeout_state();
                    set_status_leds(&mut idle_led, &mut busy_led, &mut collision_led, line_state);
                    STATE_SIGNAL.signal(line_state);
                    //info!("timeout on same output: {}", line_state);
                    if line_state == LineCondition::Collision {
                        rx_pin.wait_for_rising_edge().await;
                        current_edge.update(rx_pin.get_level());
                    }
                    //Stop receiving the transmission
                    message_complete = true;
                }
            }
        } // end of inner loop

        // lets parse
        let preamble = rx_buf[0];
        let source = rx_buf[1];
        let destination = rx_buf[2];
        let length = rx_buf[3] as usize;

        if length + 6 != bits_read / 8 {
            info!("Mismatched packet indicated length and bytes read... dropping packet");
            continue;
        }

        let crc_flag = rx_buf[4];
        let message = &rx_buf[5..5+length];
        let _crc = rx_buf[6+length];

        // drop when
        // * wrong preamble
        // * destination isn't us
        // * has a crc (for now)

        if preamble != PREAMBLE {
            info!("Incorrect Preamble: Dropping Packet");
            continue;
        }

        if destination != DEVICE_ADDRESS && destination != BROADCAST_ADDRESS {
            info!("Not our packet: Dropping");
            continue;
        }

        if crc_flag != CRC_FLAG {
            warn!("Unckecked CRC field... continuing as if passed check");
        }

        info!(
            "message finished: {}",
            core::str::from_utf8(message).unwrap()
        );
        info!("{}", rx_buf[..6+length]);

        //pwm.set_duty(embassy_stm32::timer::Channel::Ch1, max / 10);
        //Timer::after_millis(100).await;
        //pwm.set_duty(embassy_stm32::timer::Channel::Ch1, 0);

        // info!("message finished");
    } // end of outer loop
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
) 
{
    //info!("starting collision handling tx");
    let mut buf = [0u8; 320];
    let mut tx_ticker = Ticker::every(TX_HALF_BIT_PERIOD);

    loop {
        let n = reader.read(&mut buf).await;
        let mut tx_buf = [0u8; 320];
        let (tx_packet, packet_length) = match &buf[..2] {
            b"\\\\" => Packet::new(0x1e, &buf[2..n]), // changes the receive address when packet starts with "\\". this is temporary for milestone 4
            _ => Packet::new(DEVICE_ADDRESS, &buf[..n]),
        };
        tx_packet.to_u8_slice(&mut tx_buf);

        // tx_buf[..n].copy_from_slice(&buf[..n]); // might hard fault
        // tx_buf[0] = 0x55;
        //info!("Transmittig {} bytes: {}", n+1, tx_buf);

        //info!("n = {}", n);
        // let word = core::str::from_utf8(&tx_buf[..n]).unwrap();
        // let tx_bits: &BitSlice<u8, Msb0> = BitSlice::from_slice(&tx_buf[..1 + n]);
        let tx_bits: &BitSlice<u8, Msb0> = BitSlice::from_slice(&tx_buf[..packet_length]);
        //info!("received text");
        line_condition_wait_until(LineCondition::Idle).await;
        loop {
            //info!("transmitting and waiting");
            match select(
                manchester_tx(&mut tx_pin, &mut tx_ticker, &tx_bits),
                line_condition_wait_until(LineCondition::Collision),
            )
            .await
            {
                Either::First(_) => {
                    //info!("finished transmititng");
                    break;
                } // finished
                Either::Second(_) => {
                    tx_pin.set_high();
                    line_condition_wait_until(LineCondition::Idle).await;

                    // random backoff
                    let inst = Instant::now();
                    let time = inst.as_ticks() % 1000;
                    Timer::after_millis(time).await;

                    continue;
                    //info!("Stopped transmitting due to collision!");
                    //break;
                }
            }
        }
        // info!("word length {}: {}\nrecv length: {}", word.len(), word, n);
    }
}

async fn line_condition_wait_until(condition: LineCondition) {
    //info!("entering wait_until");
    while STATE_SIGNAL.wait().await != condition {
        STATE_SIGNAL.reset();
    } // exits when signal == condition
    STATE_SIGNAL.reset();
    //info!("exiting wait_until");
}

async fn manchester_tx(
    tx_pin: &mut Output<'_, AnyPin>,
    ticker: &mut Ticker,
    tx_bits: &BitSlice<u8, Msb0>,
) {
    //info!("starting tx");
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
    //info!("finished tx");
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
        //info!("waiting for uart");
        let number_characters_read = ring_uart.read(&mut read_buf).await.unwrap();
        //info!(
        //    "character received\nindex = {}\nnchar = {}",
        //    index, number_characters_read
        //);
        sending_buf[index..index + number_characters_read]
            .copy_from_slice(&read_buf[..number_characters_read]);
        index += number_characters_read;
        //info!("copy from slice successful");

        //info!("sucessfully read: {}", read_buf);
        if read_buf.contains(&b'\r') {
            let message = &sending_buf[..index - 1]; // -1 to strip carriage return
            let transmission = match message {
                NULLS_COMMAND => NULLS_PAYLOAD,
                NULLS_PRE_COMMAND => NULLS_PRE_PAYLOAD,
                AA_COMMAND => AA_PAYLOAD,
                LONG_COMMAND => LONG_PAYLOAD,
                POUND_COMMAND => POUND_PAYLOAD,
                ECONOMY_COMMAND => ECONOMY_PAYLOAD,
                FIVES_COMMAND => FIVES_PAYLOAD,
                ONES_COMMAND => ONES_PAYLOAD,
                ALAN_COMMAND_1 => ALAN_PAYLOAD_1,
                ALAN_COMMAND_2 => ALAN_PAYLOAD_2,
                _ => message,
            };
            //info!("writing to pipe");
            pipe_writer.write_all(transmission).await.unwrap();
            //info!("writing: enter pushed");
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
