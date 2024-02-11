pub(crate) mod transmit {
    use bitvec::{order::Msb0, slice::BitSlice};
    use defmt::info;
    use embassy_futures::select::{select, Either};
    use embassy_stm32::gpio::{AnyPin, Output};
    use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pipe::Reader};
    use embassy_time::Ticker;

    use crate::{LineCondition, STATE_SIGNAL};

    #[embassy_executor::task]
    pub async fn collision_handling_tx(
        mut tx_pin: Output<'static, AnyPin>,
        reader: Reader<'static, NoopRawMutex, 256>,
    ) {
        info!("starting collision handling tx");
        let mut tx_buf = [0u8; 256];
        let mut tx_ticker = Ticker::every(crate::HALF_BIT_PERIOD);

        loop {
            let n = reader.read(&mut tx_buf).await;
            info!("n = {}", n);
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
}
