#![no_std]
#![no_main]

mod transport;

use crate::transport::DynamixelSerial;
use core::cell::RefCell;
use core::ptr::addr_of_mut;
use core::time::Duration;
use critical_section::Mutex;
use defmt::Debug2Format;
use dynamixel2::{Device, Instructions, ReadError, SerialPort, TransferError};
use embedded_storage::{ReadStorage, Storage};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::cpu_control::{CpuControl, Stack};
use esp_hal::gpio::{Level, Output};
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_hal::{delay::Delay, main};
use esp_storage::FlashStorage;
use log::{error, info, warn};

const BUS_BOUDRATE: u32 = 115200;
const ID: u8 = 1;

static mut APP_CORE_STACK: Stack<8192> = Stack::new();

#[main]
fn main() -> ! {
    // generator version: 0.2.2

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_println::logger::init_logger_from_env();
    info!("Starting up");
    let common_state = Mutex::new(RefCell::new(0)); // TODO replace with actual struct

    // Store simple data in flash
    info!("Setting up persistent storage");
    let mut storage = FlashStorage::new();

    // Read data from flash from the previous run
    let mut data: [u8; 5] = [0; 5];
    storage
        .read(0, &mut data)
        .expect("Failed to read data from flash");
    info!("Data from flash: {:?}", data);

    let data: [u8; 5] = [1, 2, 3, 4, 5];
    storage
        .write(0, &data)
        .expect("Failed to write data to flash");

    // Setup UART communication
    info!("Setting up UART communication");
    let uart = Uart::new(
        peripherals.UART2,
        UartConfig::default().with_baudrate(BUS_BOUDRATE),
    )
    .expect("Failed to initialize UART controller")
    .with_rx(peripherals.GPIO21)
    .with_tx(peripherals.GPIO23);
    let mut dir_pin = Output::new(peripherals.GPIO22, Level::Low);

    // Setup the transport layer for the dynamixel communication
    info!("Setting up dynamixel communication layer");
    let transport = DynamixelSerial::new(uart, BUS_BOUDRATE, &mut dir_pin);

    info!("Setting up secondary core");
    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
    let _guard = cpu_control
        .start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, || {
            device_loop(transport, &common_state);
        })
        .unwrap();

    info!("Entering main loop on primary core");
    let delay = Delay::new();
    loop {
        // do something
        delay.delay_millis(1000);
        warn!("Main loop ran once");
    }
}

fn device_loop(transport: DynamixelSerial, common_state: &Mutex<RefCell<u32>>) -> ! {
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
            warn!(
                "unimplemented instruction: {:?}",
                Debug2Format(&instruction_catch_all)
            )
        }
    };
    Ok(())
}
