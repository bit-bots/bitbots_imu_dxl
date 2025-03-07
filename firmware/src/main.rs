#![no_std]
#![no_main]
#![feature(inline_const_pat)]

mod config;
mod imu;
mod led;
mod transport;

use crate::imu::IMUState;
use crate::transport::DynamixelSerial;
use config::ConfigManager;
use core::{cell::RefCell, ptr::addr_of_mut, time::Duration};
use critical_section::Mutex;
use defmt::Debug2Format;
use dynamixel2::{Device, Instructions, ReadError, SerialPort, TransferError};
use embedded_hal_bus::{spi::AtomicDevice, util::AtomicCell};
use esp_backtrace as _;
#[cfg(feature = "profiling")]
use esp_hal::time::now;
use esp_hal::{
    clock::CpuClock,
    cpu_control::{CpuControl, Stack},
    delay::Delay,
    gpio::{Level, Output},
    handler,
    interrupt::InterruptConfigurable,
    main, reset,
    rmt::{Rmt, TxChannel},
    spi::master::{Config as SpiConfig, Spi},
    time::RateExtU32,
    uart::{Config as UartConfig, Uart, UartInterrupt},
};
use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
use esp_storage::FlashStorage;
use heapless::Deque;
use log::{error, info, warn};

static mut APP_CORE_STACK: Stack<8192> = Stack::new();

static SERIAL: Mutex<RefCell<Option<Uart<esp_hal::Blocking>>>> = Mutex::new(RefCell::new(None));

static RX_QUEUE: Mutex<RefCell<Deque<u8, 512>>> = Mutex::new(RefCell::new(Deque::new()));

const GYRO_RANGE: f32 = 2000.0; // 2000 degrees per second
const ACCEL_RANGE: f32 = 6.0; // 6 G

const IMU_SAMPLE_RATE_HZ: u32 = 400; // Check if imu is also in the 400 Hz mode

const NUM_REG: usize = 128;
const ID_REG: usize = 7;
const BAUDRATE_REG: usize = 8;
const BAUDRATE_OPTIONS: [u32; 7] = [
    9600, 57600, 115_200, 1_000_000, 2_000_000, 3_000_000, 4_000_000,
];
const LED_START_REG: usize = 10;
const LED_REG_SIZE: usize = 4;
const NUM_LEDS: usize = 3;
const IMU_STATE_START_REG: usize = 36;

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
    let mut uart: Uart<'_, esp_hal::Blocking> = Uart::new(
        peripherals.UART2,
        UartConfig::default()
            .with_baudrate(config_manager.get(|c| c.bus_boudrate))
            .with_rx_fifo_full_threshold(1),
    )
    .expect("Failed to initialize UART controller")
    .with_rx(peripherals.GPIO21)
    .with_tx(peripherals.GPIO23);
    let mut dir_pin = Output::new(peripherals.GPIO22, Level::Low);

    uart.set_interrupt_handler(interrupt_handler);

    critical_section::with(|cs| {
        uart.listen(UartInterrupt::RxFifoFull);

        SERIAL.borrow_ref_mut(cs).replace(uart);
    });

    // Setup the transport layer for the dynamixel communication
    info!("Setting up dynamixel communication layer");

    let transport = DynamixelSerial::new(
        &SERIAL,
        config_manager.get(|c| c.bus_boudrate),
        &mut dir_pin,
    );

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

/// Reboot the device after the panic was displayed.
#[no_mangle]
pub extern "Rust" fn custom_halt() -> ! {
    error!("The chip will restart shortly!");
    // Wait so the error is definatly flushed and visible in the terminal
    let delay = Delay::new();
    delay.delay_millis(1000);
    // Reset the chip, maybe the error is gone :D
    reset::software_reset();
    unreachable!();
}

#[handler]
fn interrupt_handler() {
    critical_section::with(|cs| {
        // Get/Lock UART interface
        let mut serial = SERIAL.borrow_ref_mut(cs);
        let serial: &mut Uart<'_, esp_hal::Blocking> = serial.as_mut().unwrap();

        // Copy bytes from uart into our queue
        let mut buf = [0u8; 64];
        if let Ok(num_bytes) = serial.read_buffered_bytes(&mut buf) {
            // Push the bytes to the queue
            let mut queue = RX_QUEUE.borrow_ref_mut(cs);
            for &byte in buf.iter().take(num_bytes) {
                queue.push_back(byte).ok();
            }
        }

        // We processed this interrupt, so we can clear it
        serial.clear_interrupts(UartInterrupt::RxFifoFull.into());
    });
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
    let packet = device.read(Duration::from_micros(1000)); // TODO revert to 10

    #[cfg(feature = "profiling")]
    let profiling_t1 = now();

    if matches!(&packet,
		Err(ReadError::Io(e)) if DynamixelSerial::is_timeout_error(e))
    {
        return Ok(());
    }

    let packet = packet?;
    let device_id = config_manager.get(|c| c.id);

    // Only continue if the packet is for us or the broadcast id
    if packet.id != device_id && packet.id != 254 {
        #[cfg(feature = "profiling")]
        {
            let profiling_d1 = now() - profiling_t1;
            info!(
                "Processed packet for other device in {:?}us",
                profiling_d1.to_micros()
            );
        }
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
            // Cast the address and length to usize
            let address = address as usize;
            let length = length as usize;

            // Check if the address and length are in the range of the registers
            if address + length > NUM_REG {
                device.write_status_error(device_id, 0x07)?;
                return Ok(()); // The requested registers are out of range, but the packet was processed successfully
            }

            // Get the imu state
            let imu_buffer = critical_section::with(|cs| imu_state.borrow_ref(cs).to_le_buffer());

            // Assemble the registers
            let mut registers = [0; NUM_REG];
            registers[..2].copy_from_slice(&MODEL_NUMBER.to_le_bytes()); // u16 MODEL NUMBER
            registers[2] = FIRMWARE_VERSION; //u8 FIRMWARE VERSION
            registers[ID_REG] = config_manager.get(|c| c.id);
            registers[BAUDRATE_REG] = BAUDRATE_OPTIONS
                .iter()
                .position(|&x| x == config_manager.get(|c| c.bus_boudrate))
                .unwrap() as u8;
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
                (ID_REG, const { ID_REG + 1 }) => {
                    config_manager.set(|c| c.id = parameters[0]);
                    device.write_status_ok(device_id)?;
                }
                // Set Boudrate command
                (BAUDRATE_REG, const { BAUDRATE_REG + 1 }) => {
                    match BAUDRATE_OPTIONS.get(parameters[0] as usize) {
                        // Save the new baudrate in flash and reboot
                        Some(&baudrate) => {
                            config_manager.set(|c| c.bus_boudrate = baudrate);
                            device.write_status_ok(device_id).ok(); // Don't handle send error as we reboot either way
                            reset::software_reset();
                        }
                        // The selected baudrate is not supported
                        None => {
                            device.write_status_error(device_id, 0x07)?; // TODO fix error codes
                            return Ok(());
                        }
                    }
                }
                // Set LED state
                (
                    LED_START_REG..=const { LED_START_REG + NUM_LEDS * LED_REG_SIZE },
                    LED_START_REG..=const { LED_START_REG + NUM_LEDS * LED_REG_SIZE },
                ) => {
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
        Instructions::SyncRead {
            address: _,
            length: _,
            ids,
        } => {
            // Check if our id is in the list
            if ids.contains(&device_id) {
                warn!("The IMU does not support sync read yet");
            }
        }
        Instructions::SyncWrite {
            address: _,
            parameters: _,
            length: _,
        } => {
            warn!("The IMU does not support sync write yet")
        }
        instruction_catch_all => {
            warn!(
                "unimplemented instruction: {:?} send to id: {}",
                Debug2Format(&instruction_catch_all),
                packet.id
            )
        }
    };

    #[cfg(feature = "profiling")]
    {
        let profiling_d1 = now() - profiling_t1;
        info!("Processed packet in {:?}us", profiling_d1.to_micros());
    }

    Ok(())
}
