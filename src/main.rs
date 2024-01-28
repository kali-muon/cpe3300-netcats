#![deny(unsafe_code)]
#![no_main]
#![no_std]


// Print panic message to probe console
use panic_probe as _;


use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
};

#[allow(clippy::empty_loop)]
#[entry]
fn main() -> ! {
    loop {}
}
