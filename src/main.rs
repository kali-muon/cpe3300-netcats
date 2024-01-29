//#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

use panic_halt as _;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {
    use stm32f4xx_hal::{
        gpio::{gpioa::*, gpioc::PC13, Edge, Input, Output, PushPull},
        interrupt,
        pac::TIM3,
        prelude::*,
        timer::Event,
        timer::{CounterHz, Timer},
    };
    const SYSFREQ: u32 = 100_000_000;
    // Shared resources go here
    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    struct Local {
        led: PA5<Output>,
        tim3: CounterHz<TIM3>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        // syscfg
        let mut syscfg = ctx.device.SYSCFG.constrain();
        // clocks
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(SYSFREQ.Hz()).freeze();//.use_hse(48.MHz()).freeze();

        // counter setup
        let tim3 = ctx.device.TIM3;
        let mut led_flasher = tim3.counter_hz(&clocks);
        led_flasher.start(2.Hz()).unwrap();
        led_flasher.listen(Event::Update);

        // LED setup
        let gpioa = ctx.device.GPIOA.split();
        let mut led = gpioa.pa5.into_push_pull_output();

        // NVIC setup
        // SAFETY: it is safe because it is being unmasked during the init phase
        unsafe {
            cortex_m::peripheral::NVIC::unmask(interrupt::TIM3);
        }

        // rtic interrupt example schtuff
        // // gpio ports A and C
        // let gpioa = ctx.device.GPIOA.split();
        // let gpioc = ctx.device.GPIOC.split();
        // // button
        // let mut button = gpioa.pa0.into_pull_up_input();
        // button.make_interrupt_source(&mut syscfg);
        // button.enable_interrupt(&mut ctx.device.EXTI);
        // button.trigger_on_edge(&mut ctx.device.EXTI, Edge::Falling);
        // // led
        // let led = gpioc.pc13.into_push_pull_output();

        (
            Shared {
               // Initialization of shared resources go here
            },
            Local {
                // Initialization of local resources go here
                led: led,
                tim3: led_flasher,
            },
        )
    }

    #[task(binds = TIM3, local = [led, tim3])]
    fn blink_led_isr(ctx: blink_led_isr::Context) {
        ctx.local.tim3.clear_all_flags();
        ctx.local.led.toggle();
    }

    // Optional idle, can be removed if not needed.
    //#[idle]
    //fn idle(_: idle::Context) -> ! {
    //    loop {
    //        continue;
    //    }
    //}
}
