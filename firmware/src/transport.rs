use core::time::Duration;
use dynamixel2::SerialPort;
use embedded_io::Write;
use esp_hal::{
    delay::Delay,
    gpio::Output,
    time::{now, ExtU64, Instant},
    uart::{Error as UartError, Uart},
    Blocking,
};

#[derive(Debug)]
#[allow(dead_code)]
pub enum Error {
    UartReadError(UartError), // TODO look at error handling
    UartWriteError(UartError),
    Timeout,
}
pub struct DynamixelSerial<'d> {
    serial: Uart<'d, Blocking>,
    baud_rate: u32,
    dir: &'d mut Output<'d>,
}

impl<'d> DynamixelSerial<'d> {
    pub fn new(serial: Uart<'d, Blocking>, baud_rate: u32, dir: &'d mut Output<'d>) -> Self {
        // Set direction pin to low just to be sure
        dir.set_low();
        // Initialize the struct
        Self {
            serial,
            baud_rate,
            dir,
        }
    }
}

impl SerialPort for DynamixelSerial<'_> {
    type Error = Error;
    type Instant = Instant;

    fn baud_rate(&self) -> Result<u32, Self::Error> {
        Ok(self.baud_rate)
    }

    fn set_baud_rate(&mut self, _baud_rate: u32) -> Result<(), Self::Error> {
        panic!("Changing baud rate is not supported");
    }

    fn discard_input_buffer(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn read(&mut self, buffer: &mut [u8], deadline: &Self::Instant) -> Result<usize, Self::Error> {
        ///let delay = Delay::new();
        while deadline > &now() {
            let num_bytes_available = self
                .serial
                .read_buffered_bytes(buffer)
                .map_err(Error::UartReadError)?;
            if num_bytes_available > 0 {
                return Ok(num_bytes_available);
            }
            // Retry if no bytes are available
            //delay.delay_micros(10); // TODO check if this is the right delay
        }
        Err(Error::Timeout)
    }

    fn write_all(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
        self.dir.set_high();
        self.serial
            .write_all(buffer)
            .map_err(Error::UartWriteError)?;
        Write::flush(&mut self.serial).map_err(Error::UartWriteError)?;
        self.dir.set_low();
        Ok(())
    }

    fn make_deadline(&self, timeout: Duration) -> Self::Instant {
        let timeout: u64 = timeout.as_millis() as u64;
        now() + timeout.millis()
    }

    fn is_timeout_error(error: &Self::Error) -> bool {
        matches!(error, Error::Timeout)
    }
}
