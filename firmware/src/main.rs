#![no_std]
#![no_main]

mod transport;

use crate::transport::DynamixelSerial;
use core::time::Duration;
use defmt::Debug2Format;
use dynamixel2::{Device, Instructions, ReadError, SerialPort, TransferError};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::main;
use esp_hal::uart::{Config as UartConfig, Uart};
use log::{error, info, warn};

extern crate alloc;

const BUS_BOUDRATE: u32 = 115200;
const ID: u8 = 1;

#[main]
fn main() -> ! {
    // generator version: 0.2.2

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_println::logger::init_logger_from_env();
    esp_alloc::heap_allocator!(72 * 1024);

    let (rx_pin, tx_pin) = (peripherals.GPIO16, peripherals.GPIO17);

    let uart = Uart::new(
        peripherals.UART2,
        UartConfig::default().with_baudrate(BUS_BOUDRATE),
    )
    .expect("Failed to initialize UART controller")
    .with_rx(rx_pin)
    .with_tx(tx_pin);

    let transport = DynamixelSerial::new(uart, BUS_BOUDRATE);

    device_loop(transport);

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.23.1/examples/src/bin
}

fn device_loop(transport: DynamixelSerial) -> ! {
    let mut device = Device::with_buffers(transport, [0; 200], [0; 200])
        .expect("Failed to initialize dynamixel device");
    loop {
        info!("Waiting for packet");
        if let Err(e) = process_packet(&mut device) {
            error!("{:?}", Debug2Format(&e))
        }
    }
}

fn process_packet<ReadBuffer, WriteBuffer>(
    device: &mut Device<ReadBuffer, WriteBuffer, DynamixelSerial>,
) -> Result<(), TransferError<transport::Error>>
where
    WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
    ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
{
    let packet = device.read(Duration::from_millis(10));
    if matches!(&packet,
		Err(ReadError::Io(e)) if DynamixelSerial::is_timeout_error(&e))
    {
        info!("Timeout");
        return Ok(());
    }
    let packet = packet?;
    let id = packet.id;
    if id != ID && id != 254 {
        // 254 is the broadcast id (?)
        return Ok(());
    }
    match packet.instruction {
        Instructions::Ping => {
            // todo: this should wait for based on id for some amount of time
            device.write_status(ID, 0, 3, |buffer| {
                buffer[..2].copy_from_slice(&1020_u16.to_le_bytes()); // u16 MODEL NUMBER
                buffer[2] = 1; //u8 FIRMWARE VERSION
            })?;
        }
        Instructions::Read { address, length } => {
            if let Some(data) = todo!("get your data for reading") {
                device.write_status(ID, 0, length as usize, |buffer| {
                    buffer.copy_from_slice(data);
                })?;
            } else {
                device.write_status_error(ID, 0x07)?;
            }
        }
        Instructions::Write {
            address,
            parameters,
        } => {
            if todo!("perform the write") {
                device.write_status_ok(ID)?;
            } else {
                device.write_status_error(ID, 0x07)?;
            }
        }
        Instructions::Unknown { instruction, .. } => {
            error!("Unknown instruction {:?}", instruction)
        }
        instruction_catch_all => {
            warn!("unimplemented instruction: {:?}", Debug2Format(&instruction_catch_all))
        }
    };
    Ok(())
}
