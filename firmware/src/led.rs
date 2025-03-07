use esp_hal::rmt::TxChannel;
use esp_hal_smartled::SmartLedsAdapter;
use smart_leds::{brightness, gamma, SmartLedsWrite, RGB8};

pub struct LedComponent<'a, C: TxChannel, const BUFFER_SIZE: usize> {
    pub state: [u8; crate::LED_REG_SIZE * crate::NUM_LEDS],
    driver: &'a mut SmartLedsAdapter<C, BUFFER_SIZE>,
}

impl<'a, C: TxChannel, const BUFFER_SIZE: usize> LedComponent<'a, C, BUFFER_SIZE> {
    pub fn new(driver: &'a mut SmartLedsAdapter<C, BUFFER_SIZE>) -> Self {
        Self {
            state: [0; crate::LED_REG_SIZE * crate::NUM_LEDS],
            driver,
        }
    }

    pub fn send(&mut self) {
        let state = self.state;
        let colors = (0..crate::NUM_LEDS).map(|i| {
            let led_index = i * crate::LED_REG_SIZE;
            RGB8 {
                r: state[led_index],
                g: state[led_index + 1],
                b: state[led_index + 2],
            }
        });
        self.driver.write(brightness(gamma(colors), 10)).unwrap();
    }
}
