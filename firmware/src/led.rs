


struct LEDComponent {
    driver: String,
    state: String,
}

impl LEDComponent {
    fn new(rmt: Rmt, pin: u32) -> Self {
        Self {
            driver,
            state,
        }
    }
}
