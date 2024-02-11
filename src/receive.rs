pub(crate) mod receive {
    use embassy_stm32::gpio::{AnyPin, Output};

    use crate::LineCondition;

    pub fn set_status_leds(
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
}
