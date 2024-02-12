pub(crate) mod receive {
    use bitvec::{order::Msb0, slice::BitSlice};
    use defmt::info;
    use embassy_futures::select::{select, Either};
    use embassy_stm32::{
        exti::ExtiInput,
        gpio::{AnyPin, Level, Output},
        peripherals::PB8,
    };
    use embassy_time::{Instant, Timer};

    use crate::{
        LineCondition, BIT_PERIOD_LOWER_TOLERANCE, BIT_PERIOD_UPPER_TOLERANCE, COLLISION_CUTOFF,
        HALF_BIT_PERIOD_LOWER_TOLERANCE, HALF_BIT_PERIOD_UPPER_TOLERANCE, IDLE_CUTOFF,
        STATE_SIGNAL,
    };

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

    #[embassy_executor::task]
    pub async fn receive_logic(
        mut idle_led: &mut Output<'static, AnyPin>,
        mut busy_led: &mut Output<'static, AnyPin>,
        mut collision_led: &mut Output<'static, AnyPin>,
        rx_pin: &mut ExtiInput<'static, PB8>,
    ) {
        let mut line_state = LineCondition::Idle;
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
        info!("entering outer loop");
        loop {
            info!("top of outer loop");
            // loops once for each packet
            let mut rx_buf = [0u8; 320];
            let rx_bits: &mut BitSlice<_, Msb0> = BitSlice::from_slice_mut(&mut rx_buf);
            let mut rx_iter = rx_bits.iter_mut();
            let mut message_complete: bool = false;
            {
                // "begin transmission" state
                // Assume we begin in idle state
                info!("waiting for message to start");
                rx_pin.wait_for_falling_edge().await;
                previous_edge.update(rx_pin.get_level());
                line_state = LineCondition::Busy;
                set_status_leds(&mut idle_led, &mut busy_led, &mut collision_led, line_state);
                STATE_SIGNAL.signal(line_state);
                info!("got the first falling edge");
                let mut synchronized: bool = false;
                // now we are out of idle
                while !synchronized && !message_complete {
                    let timeout_time = previous_edge.get_timeout_time(); // sets the collision cutoff
                    match select(rx_pin.wait_for_any_edge(), Timer::at(timeout_time)).await {
                        Either::First(_) => {
                            current_edge.update(rx_pin.get_level());
                            if current_edge.time.duration_since(previous_edge.time)
                                > BIT_PERIOD_LOWER_TOLERANCE
                            {
                                previous_edge = current_edge;
                                read_bit = current_edge.level;
                                // exit "begin transmission state"
                                synchronized = true;
                                info!("synchronized");
                                // enter "decoding state"
                            } else {
                                info!("not synchronized yet");
                                // stay in "begin transmission state"
                            }
                        }
                        Either::Second(_) => {
                            line_state = previous_edge.get_timeout_state();
                            set_status_leds(
                                &mut idle_led,
                                &mut busy_led,
                                &mut collision_led,
                                line_state,
                            );
                            STATE_SIGNAL.signal(line_state);
                            // stop reception of message
                            message_complete = true;
                            info!(
                                "collision while synchronizing\nare we collision?: {}",
                                (line_state == LineCondition::Collision)
                            );
                        }
                    };
                }
            }
            if !message_complete {
                info!("entering decoding loop");
            }
            while !message_complete {
                // loops while receiving a packet

                // decoding state -----------------------------------------------------
                // Wait for an edge
                info!("waiting for next bit");
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
                            && current_edge.time.duration_since(previous_edge.time)
                                < BIT_PERIOD_UPPER_TOLERANCE
                        {
                            //Send the line value
                            rx_iter.next().unwrap().commit(current_edge.level.into());
                            info!("wrote bit: {}", current_edge.level);
                            //Cycle the edge for next decode
                            current_edge = previous_edge;
                            //Loop back to beginning
                        }
                        //T since last edge
                        else if current_edge.time.duration_since(previous_edge.time)
                            > HALF_BIT_PERIOD_LOWER_TOLERANCE
                            && current_edge.time.duration_since(previous_edge.time)
                                < HALF_BIT_PERIOD_UPPER_TOLERANCE
                        {
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
                                    if current_edge.time.duration_since(previous_edge.time)
                                        > HALF_BIT_PERIOD_LOWER_TOLERANCE
                                        && current_edge.time.duration_since(previous_edge.time)
                                            < HALF_BIT_PERIOD_UPPER_TOLERANCE
                                    {
                                        //We're safe, received half a bit period

                                        //Send the line value
                                        rx_iter.next().unwrap().commit(current_edge.level.into());
                                        info!("wrote bit: {}", current_edge.level);
                                        //Cycle the edge for next decode
                                        current_edge = previous_edge;
                                        //Loop back to beginning
                                    } else {
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
                                    info!("timeout trying to switch value: {}", line_state);
                                    //Stop receiving the transmission
                                    message_complete = true;
                                }
                            }
                        }
                        //Non-valid time since last edge
                        else {
                            //Invalid data, stop reception
                        }
                    }

                    Either::Second(_) => {
                        line_state = previous_edge.get_timeout_state();
                        set_status_leds(
                            &mut idle_led,
                            &mut busy_led,
                            &mut collision_led,
                            line_state,
                        );
                        STATE_SIGNAL.signal(line_state);
                        info!("timeout on same output: {}", line_state);
                        //Stop receiving the transmission
                        message_complete = true;
                    }
                }
            } // end of inner loop
              // info!("message finished: {}", rx_buf);
            info!("message finished");
        } // end of outer loop
    }
}
