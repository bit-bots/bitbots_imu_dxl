#![no_std]
#![no_main]

mod config;
mod imu;
mod led;
mod transport;

use crate::imu::IMUState;
use crate::transport::DynamixelSerial;
use config::ConfigManager;
use core::{
    cell::RefCell,
    fmt,
    ptr::addr_of_mut,
    time::{self, Duration},
};
use critical_section::Mutex;
use defmt::Debug2Format;
use dynamixel2::{Device, Instructions, ReadError, SerialPort, TransferError};
use embedded_hal_bus::{spi::AtomicDevice, util::AtomicCell};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    cpu_control::{CpuControl, Stack},
    delay::Delay,
    gpio::{Level, Output},
    main, reset,
    rmt::{Rmt, TxChannel},
    spi::master::{Config as SpiConfig, Spi},
    time::{now, RateExtU32},
    uart::{Config as UartConfig, Uart},
};
use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
use esp_storage::FlashStorage;
use log::{error, info, warn};

static mut APP_CORE_STACK: Stack<8192> = Stack::new();

const GYRO_RANGE: f32 = 2000.0; // 2000 degrees per second
const ACCEL_RANGE: f32 = 6.0; // 6 G

const IMU_SAMPLE_RATE_HZ: u32 = 400; // Check if imu is also in the 400 Hz mode

const ID_REG: usize = 7;
const BAUDRATE_REG: usize = 8;
const NUM_LEDS: usize = 3;
const LED_START_REG: usize = 10;
const LED_REG_SIZE: usize = 4;
const LED_REG_END: usize = LED_START_REG + NUM_LEDS * LED_REG_SIZE;

const MODEL_NUMBER: u16 = 0xBAFF;
const FIRMWARE_VERSION: u8 = 1;

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_println::logger::init_logger_from_env();
    info!("Starting up");

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

    // Setup LEDs
    let rmt = Rmt::new(peripherals.RMT, 80.MHz()).unwrap();
    let mut led_driver =
        SmartLedsAdapter::new(rmt.channel0, peripherals.GPIO27, smartLedBuffer!(3));
    let led = led::LedComponent::new(&mut led_driver);

    // Setup the IMU Device
    info!("Setting up IMU");

    let spi = AtomicCell::new(
        Spi::new(peripherals.SPI2, SpiConfig::default())
            .unwrap()
            .with_sck(peripherals.GPIO19)
            .with_mosi(peripherals.GPIO5)
            .with_miso(peripherals.GPIO17),
    );

    let mut delay = Delay::new();

    let mut gyro_device = bmi088::Builder::new_gyro_spi(
        AtomicDevice::new(&spi, Output::new(peripherals.GPIO18, Level::High), delay).unwrap(),
    );
    gyro_device.setup(&mut delay).unwrap();

    let mut accel_device = bmi088::Builder::new_accel_spi(
        AtomicDevice::new(&spi, Output::new(peripherals.GPIO26, Level::High), delay).unwrap(),
    );
    accel_device.setup(&mut delay).unwrap();

    let imu_state = Mutex::new(RefCell::new(IMUState::default()));

    info!("Setting up secondary core");
    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
    // This is needed to the imu_state is borrowed in the closure
    let imu_state_ref = &imu_state;
    // Move everything into the closure
    let core_2_closure = move || {
        imu::filter_loop(imu_state_ref, gyro_device, accel_device);
    };
    // Start the secondary core
    let _guard = cpu_control
        .start_app_core(
            unsafe { &mut *addr_of_mut!(APP_CORE_STACK) },
            core_2_closure,
        )
        .unwrap();

    // Spin
    device_loop(transport, &imu_state, led, &config_manager);
}

fn device_loop<LEDC: TxChannel, const LED_BUFFER_SIZE: usize>(
    transport: DynamixelSerial,
    imu_state: &Mutex<RefCell<IMUState>>,
    mut led: led::LedComponent<LEDC, LED_BUFFER_SIZE>,
    config_manager: &ConfigManager,
) -> ! {
    let mut device = Device::with_buffers(transport, [0; 200], [0; 200])
        .expect("Failed to initialize dynamixel device");
    loop {
        if let Err(e) = process_packet(&mut device, imu_state, &mut led, config_manager) {
            error!("{:?}", Debug2Format(&e))
        }
    }
}

fn process_packet<ReadBuffer, WriteBuffer, LEDC: TxChannel, const LED_BUFFER_SIZE: usize>(
    device: &mut Device<ReadBuffer, WriteBuffer, DynamixelSerial>,
    imu_state: &Mutex<RefCell<IMUState>>,
    led: &mut led::LedComponent<LEDC, LED_BUFFER_SIZE>,
    config_manager: &ConfigManager,
) -> Result<(), TransferError<transport::Error>>
where
    WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
    ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
{
    let packet = device.read(Duration::from_millis(1)); // TODO revert to 10
    let time1 = now();
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
        let time2 = now();
        let duration = time2 - time1;
        info!("Processed packet in {:?}ys", duration.to_micros());
        // 254 is the broadcast id (?)
        return Ok(());
    }

    // Handle the different instructions
    match packet.instruction {
        Instructions::Ping => {
            info!("Ping");
            // todo: this should wait for based on id for some amount of time
            device.write_status(device_id, 0, 3, |buffer| {
                buffer[..2].copy_from_slice(&MODEL_NUMBER.to_le_bytes()); // u16 MODEL NUMBER
                buffer[2] = FIRMWARE_VERSION; //u8 FIRMWARE VERSION
            })?;
        }
        Instructions::Read { address, length } => {
            const IMU_STATE_START_REG: usize = 36;
            const NUM_REG: usize = 128;

            // Cast the address and length to usize
            let address = address as usize;
            let length = length as usize;

            // Check if the address and length are in the range of the registers
            if address + length > NUM_REG {
                device.write_status_error(device_id, 0x07)?;
                return Ok(()); // The requested registers are out of range, but the packet was processed successfully
            }

            // Get the imu state
            let imu_buffer =
                critical_section::with(|cs| imu_state.borrow_ref(cs).clone()).to_le_buffer();

            // Assemble the registers
            let mut registers = [0; NUM_REG];
            registers[..2].copy_from_slice(&MODEL_NUMBER.to_le_bytes()); // u16 MODEL NUMBER
            registers[2] = FIRMWARE_VERSION; //u8 FIRMWARE VERSION
            registers[ID_REG] = config_manager.get(|c| c.id);
            registers[BAUDRATE_REG] = match config_manager.get(|c| c.bus_boudrate) {
                9600 => 0,
                57600 => 1,
                115200 => 2,
                1000000 => 3,
                2000000 => 4,
                3000000 => 5,
                4000000 => 6,
                _ => unreachable!(),
            };
            registers[IMU_STATE_START_REG..IMU_STATE_START_REG + imu_buffer.len()]
                .copy_from_slice(&imu_buffer);
            // TODO other registers

            // Answer the read request
            device.write_status(device_id, 0, length, |buffer| {
                buffer.copy_from_slice(&registers[address..address + length]);
            })?;
        }
        Instructions::Write {
            address,
            parameters,
        } => {
            let address = address as usize;
            let length = parameters.len();
            let end_address = address + length;

            match (address, end_address) {
                // Set ID command
                (ID_REG, _) => {
                    config_manager.set(|c| c.id = parameters[0]);
                    device.write_status_ok(device_id)?;
                }
                // Set Boudrate command
                (BAUDRATE_REG, _) => {
                    let baudrate = match parameters[0] {
                        0 => 9600,
                        1 => 57600,
                        2 => 115200,
                        3 => 1000000,
                        4 => 2000000,
                        5 => 3000000,
                        6 => 4000000,
                        _ => {
                            device.write_status_error(device_id, 0x07)?; // TODO fix error codes
                            return Ok(());
                        }
                    };
                    config_manager.set(|c| c.bus_boudrate = baudrate);
                    device.write_status_ok(device_id)?;
                    reset::software_reset();
                }
                // Set LED state
                (LED_START_REG..=LED_REG_END, _) => {
                    // Update the led state
                    led.state[address - LED_START_REG..address - LED_START_REG + length]
                        .copy_from_slice(parameters);
                    led.send();
                    device.write_status_ok(device_id)?;
                }
                _ => {
                    device.write_status_error(device_id, 0x07)?;
                }
            }
        }
        Instructions::Unknown { instruction, .. } => {
            error!("Unknown instruction {:?}", instruction)
        }
        Instructions::Reboot => {
            warn!("Reboot triggered over DXL");
            reset::software_reset();
        }
        instruction_catch_all => {
            warn!(
                "unimplemented instruction: {:?}",
                Debug2Format(&instruction_catch_all)
            )
        }
    };
    let time2 = now();
    let duration = time2 - time1;
    info!("Processed packet in {:?}ys", duration.to_micros());
    Ok(())
}
