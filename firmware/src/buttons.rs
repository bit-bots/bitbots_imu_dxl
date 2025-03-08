use esp_hal::{
    gpio::{Input, InputPin, Pull},
    peripheral::Peripheral,
};

use crate::NUM_BUTTONS;

pub struct ButtonComponent<'a> {
    pin_0: Input<'a>,
    pin_1: Input<'a>,
    pin_2: Input<'a>,
}

impl<'a> ButtonComponent<'a> {
    pub fn new(
        pin_0: impl Peripheral<P = impl InputPin + 'a> + 'a,
        pin_1: impl Peripheral<P = impl InputPin + 'a> + 'a,
        pin_2: impl Peripheral<P = impl InputPin + 'a> + 'a,
    ) -> Self {
        Self {
            pin_0: Input::new(pin_0, Pull::Up),
            pin_1: Input::new(pin_1, Pull::Up),
            pin_2: Input::new(pin_2, Pull::Up),
        }
    }

    pub fn read_bool(&self) -> [bool; NUM_BUTTONS] {
        [
            self.pin_0.is_low(),
            self.pin_1.is_low(),
            self.pin_2.is_low(),
        ]
    }

    pub fn read_u8(&self) -> [u8; NUM_BUTTONS] {
        self.read_bool().map(|b| b as u8)
    }
}
