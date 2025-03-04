#![no_std]
#![no_main]

mod config;
mod transport;

use crate::transport::DynamixelSerial;
use config::ConfigManager;
use core::{cell::RefCell, ptr::addr_of_mut, time::Duration};
use critical_section::Mutex;
use defmt::Debug2Format;
use dynamixel2::{Device, Instructions, ReadError, SerialPort, TransferError};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    cpu_control::{CpuControl, Stack},
    delay::Delay,
    gpio::{Level, Output},
    main,
    uart::{Config as UartConfig, Uart},
    reset
};
use esp_storage::FlashStorage;
use log::{error, info, warn};

static mut APP_CORE_STACK: Stack<8192> = Stack::new();

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_println::logger::init_logger_from_env();
    info!("Starting up");
    let common_state = Mutex::new(RefCell::new(0)); // TODO replace with actual struct

    // Store simple data in flash
    info!("Loading config from persistent storage");
    let flash = FlashStorage::new();
    let config_manager = ConfigManager::new(flash, 30);

    // Setup UART communication
    info!("Setting up UART communication");
    let uart = Uart::new(
        peripherals.UART2,
        UartConfig::default().with_baudrate(config_manager.get(|c| c.bus_boudrate)),
    )
    .expect("Failed to initialize UART controller")
    .with_rx(peripherals.GPIO21)
    .with_tx(peripherals.GPIO23);
    let mut dir_pin = Output::new(peripherals.GPIO22, Level::Low);

    // Setup the transport layer for the dynamixel communication
    info!("Setting up dynamixel communication layer");
    let transport =
        DynamixelSerial::new(uart, config_manager.get(|c| c.bus_boudrate), &mut dir_pin);

    info!("Setting up secondary core");
    //let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
    //let _guard = cpu_control
    //    .start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, || {
    //        filter_loop(&common_state)
    //    })
    //    .unwrap();

    device_loop(transport, &common_state, &config_manager);
}

fn filter_loop(common_state: &Mutex<RefCell<u32>>) -> ! {
    let delay = Delay::new();

    loop {
        delay.delay_millis(100);
        info!("Secondary core loop");
    }
}

fn device_loop(
    transport: DynamixelSerial,
    common_state: &Mutex<RefCell<u32>>,
    config_manager: &ConfigManager,
) -> ! {
    let mut device = Device::with_buffers(transport, [0; 200], [0; 200])
        .expect("Failed to initialize dynamixel device");
    loop {
        info!("Waiting for packet");
        if let Err(e) = process_packet(&mut device, common_state, config_manager) {
            error!("{:?}", Debug2Format(&e))
        }
    }
}

fn process_packet<ReadBuffer, WriteBuffer>(
    device: &mut Device<ReadBuffer, WriteBuffer, DynamixelSerial>,
    common_state: &Mutex<RefCell<u32>>,
    config_manager: &ConfigManager,
) -> Result<(), TransferError<transport::Error>>
where
    WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
    ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
{
    let packet = device.read(Duration::from_millis(1000)); // TODO revert to 10
    if matches!(&packet,
		Err(ReadError::Io(e)) if DynamixelSerial::is_timeout_error(&e))
    {
        info!("Timeout");
        return Ok(());
    }

    let packet = packet?;
    let device_id = config_manager.get(|c| c.id);

    // Check if the packet is for us
    if packet.id != device_id && packet.id != 254 {
        // 254 is the broadcast id (?)
        return Ok(());
    }

    // Handle the different instructions
    match packet.instruction {
        Instructions::Ping => {
            info!("Ping");
            // todo: this should wait for based on id for some amount of time
            device.write_status(device_id, 0, 3, |buffer| {
                buffer[..2].copy_from_slice(&43962_u16.to_le_bytes()); // u16 MODEL NUMBER
                buffer[2] = 1; //u8 FIRMWARE VERSION
            })?;
        }
        Instructions::Read { address, length } => {
            if let Some(data) = todo!("get your data for reading") {
                device.write_status(device_id, 0, length as usize, |buffer| {
                    buffer.copy_from_slice(data);
                })?;
            } else {
                device.write_status_error(device_id, 0x07)?;
            }
        }
        Instructions::Write {
            address,
            parameters,
        } => {
            if todo!("perform the write") {
                device.write_status_ok(device_id)?;
            } else {
                device.write_status_error(device_id, 0x07)?;
            }
        }
        Instructions::Unknown { instruction, .. } => {
            error!("Unknown instruction {:?}", instruction)
        }
        Instructions::Reboot => {
            info!("Reboot");
            reset::software_reset();
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
