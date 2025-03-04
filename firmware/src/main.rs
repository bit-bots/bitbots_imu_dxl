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
use embedded_hal_bus::spi::RefCellDevice;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    cpu_control::{CpuControl, Stack},
    delay::Delay,
    gpio::{Level, Output},
    ledc::{LSGlobalClkSource, Ledc},
    main, reset,
    rmt::Rmt,
    spi::{
        self,
        master::{Config as SpiConfig, Spi},
    },
    time::RateExtU32,
    uart::{Config as UartConfig, Uart},
};
use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
use esp_storage::FlashStorage;
use log::{error, info, warn};
use smart_leds::{
    brightness, gamma,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite, RGB8,
};

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

    // Setup LEDs
    let rmt = Rmt::new(peripherals.RMT, 80.MHz()).unwrap();

    let rmt_buffer = smartLedBuffer!(3);
    let mut led = SmartLedsAdapter::new(rmt.channel0, peripherals.GPIO27, rmt_buffer);

    let delay = Delay::new();

    fn get_color(hue: u8) -> RGB8 {
        hsv2rgb(Hsv {
            hue,
            sat: 255,
            val: 255,
        })
    }

    let mut data;

    loop {
        // Iterate over the rainbow!
        for hue in 0..=255 {
            // Convert from the HSV color space (where we can easily transition from one
            // color to the other) to the RGB color space that we can then send to the LED
            data = [
                get_color(hue),
                get_color((hue + 255 / 3) % 255),
                get_color((hue + 2 * (255 / 3)) % 255),
            ];
            // When sending to the LED, we do a gamma correction first (see smart_leds
            // documentation for details) and then limit the brightness to 10 out of 255 so
            // that the output it's not too bright.
            led.write(brightness(gamma(data.iter().cloned()), 10))
                .unwrap();
            delay.delay_millis(20);
        }
    }

    // Setup the IMU
    info!("Setting up IMU");

    let spi = RefCell::new(
        Spi::new(peripherals.SPI2, SpiConfig::default())
            .unwrap()
            .with_sck(peripherals.GPIO19)
            .with_mosi(peripherals.GPIO5)
            .with_miso(peripherals.GPIO17),
    );

    let mut delay = Delay::new();

    let accel_device =
        RefCellDevice::new(&spi, Output::new(peripherals.GPIO26, Level::High), delay).unwrap();

    let gyro_device =
        RefCellDevice::new(&spi, Output::new(peripherals.GPIO18, Level::High), delay).unwrap();

    let mut bmi088_g = bmi088::Builder::new_gyro_spi(gyro_device);
    bmi088_g.setup(&mut delay).unwrap();

    let mut bmi088_a = bmi088::Builder::new_accel_spi(accel_device);
    bmi088_a.setup(&mut delay).unwrap();

    loop {
        if let Ok(gyro_sample) = bmi088_g.get_gyro() {
            info!("bmi088_g: {:?}", gyro_sample);
        }

        if let Ok(accel_sample) = bmi088_a.get_accel() {
            info!("bmi088_a: {:?}", accel_sample);
        }
    }

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
