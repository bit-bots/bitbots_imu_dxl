#![no_std]
#![no_main]
#![feature(inline_const_pat)]

mod buttons;
mod config;
mod device;
mod imu;
mod led;
mod transport;

use core::{cell::RefCell, ptr::addr_of_mut};

use config::ConfigManager;
use critical_section::Mutex;
use embedded_hal_bus::{spi::AtomicDevice, util::AtomicCell};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    cpu_control::{CpuControl, Stack},
    delay::Delay,
    gpio::{Level, Output},
    handler,
    interrupt::InterruptConfigurable,
    main, reset,
    rmt::Rmt,
    spi::master::{Config as SpiConfig, Spi},
    time::RateExtU32,
    uart::{Config as UartConfig, Uart, UartInterrupt},
};
use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
use esp_storage::FlashStorage;
use heapless::Deque;
use log::{error, info};

use crate::buttons::ButtonComponent;
use crate::imu::IMUState;
use crate::led::LedComponent;
use crate::transport::DynamixelSerial;

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
const NUM_BUTTONS: usize = 3;
const BUTTON_START_REG: usize = 76;

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
    // Setup UART interrupt
    uart.set_interrupt_handler(uart_handler);
    critical_section::with(|cs| {
        uart.listen(UartInterrupt::RxFifoFull);
        SERIAL.borrow_ref_mut(cs).replace(uart);
    });
    // Setup UART dir pin
    let mut dir_pin = Output::new(peripherals.GPIO22, Level::Low);

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
    let led = LedComponent::new(&mut led_driver);

    // Setup Buttons
    let button_driver =
        ButtonComponent::new(peripherals.GPIO2, peripherals.GPIO32, peripherals.GPIO4);

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
    device::device_loop(transport, &imu_state, led, button_driver, &config_manager);
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
fn uart_handler() {
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
