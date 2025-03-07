use core::{cell::RefCell, time::Duration};

use critical_section::Mutex;
use dynamixel2::SerialPort;
use embedded_io::Write;
use esp_hal::{
    gpio::Output,
    time::{now, ExtU64, Instant},
    uart::{Error as UartError, Uart},
};

#[derive(Debug)]
#[allow(dead_code)]
pub enum Error {
    UartRead(UartError), // TODO look at error handling
    UartWrite(UartError),
    Timeout,
}
pub struct DynamixelSerial<'d, 'e> {
    serial: &'d Mutex<RefCell<Option<Uart<'d, esp_hal::Blocking>>>>,
    baud_rate: u32,
    dir: &'e mut Output<'e>,
}

impl<'d, 'e> DynamixelSerial<'d, 'e> {
    pub fn new(
        serial: &'d Mutex<RefCell<Option<Uart<'d, esp_hal::Blocking>>>>,
        baud_rate: u32,
        dir: &'e mut Output<'e>,
    ) -> Self {
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

impl SerialPort for DynamixelSerial<'_, '_> {
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
        while deadline > &now() {
            let num_bytes_read = critical_section::with(|cs| {
                // Lock queue
                let mut queue = crate::RX_QUEUE.borrow_ref_mut(cs);

                // Copy data from queue to buffer
                if let Some(byte) = queue.pop_front() {
                    buffer[0] = byte;
                    return 1;
                }
                0
            });
            if num_bytes_read > 0 {
                return Ok(num_bytes_read);
            }
        }
        Err(Error::Timeout)
    }

    fn write_all(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
        critical_section::with(|cs| {
            self.dir.set_high();
            let mut serial = self.serial.borrow_ref_mut(cs);
            let serial = serial.as_mut().unwrap();

            // Encapsulate write and flush together
            // That way we can reset the dir pin even if they fail
            let result = || -> Result<(), UartError> {
                serial.write_all(buffer)?;
                Write::flush(serial)
            }()
            .map_err(Error::UartWrite);

            self.dir.set_low();
            result
        })
    }

    fn make_deadline(&self, timeout: Duration) -> Self::Instant {
        let timeout: u64 = timeout.as_millis() as u64;
        now() + timeout.millis()
    }

    fn is_timeout_error(error: &Self::Error) -> bool {
        matches!(error, Error::Timeout)
    }
}
