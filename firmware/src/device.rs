use core::{cell::RefCell, time::Duration};

use critical_section::Mutex;
use defmt::Debug2Format;
use dynamixel2::{Device, Instructions, ReadError, SerialPort, TransferError};
use esp_backtrace as _;
#[cfg(feature = "profiling")]
use esp_hal::time::now;
use esp_hal::{reset, rmt::TxChannel};
use log::{error, info, warn};

use crate::{
    buttons::ButtonComponent,
    config::ConfigManager,
    imu::IMUState,
    led,
    transport::{self, DynamixelSerial},
    BAUDRATE_OPTIONS, BAUDRATE_REG, BUTTON_START_REG, FIRMWARE_VERSION, ID_REG,
    IMU_STATE_START_REG, LED_REG_SIZE, LED_START_REG, MODEL_NUMBER, NUM_BUTTONS, NUM_LEDS, NUM_REG,
};

pub fn device_loop<LEDC: TxChannel, const LED_BUFFER_SIZE: usize>(
    transport: DynamixelSerial,
    imu_state: &Mutex<RefCell<IMUState>>,
    mut led: led::LedComponent<LEDC, LED_BUFFER_SIZE>,
    mut buttons: ButtonComponent,
    config_manager: &ConfigManager,
) -> ! {
    let mut device = Device::with_buffers(transport, [0; 200], [0; 200])
        .expect("Failed to initialize dynamixel device");
    loop {
        if let Err(e) = process_packet(
            &mut device,
            imu_state,
            &mut led,
            &mut buttons,
            config_manager,
        ) {
            error!("{:?}", Debug2Format(&e))
        }
    }
}

fn process_packet<Buffer, LEDC: TxChannel, const LED_BUFFER_SIZE: usize>(
    device: &mut Device<DynamixelSerial, Buffer>,
    imu_state: &Mutex<RefCell<IMUState>>,
    led: &mut led::LedComponent<LEDC, LED_BUFFER_SIZE>,
    buttons: &mut ButtonComponent,
    config_manager: &ConfigManager,
) -> Result<(), TransferError<transport::Error>>
where
    Buffer: AsRef<[u8]> + AsMut<[u8]>,
{
    let packet = device.read(Duration::from_micros(1000)); // TODO revert to 10

    #[cfg(feature = "profiling")]
    let profiling_t1 = now();

    if matches!(&packet,
		Err(ReadError::Io(e)) if DynamixelSerial::is_timeout_error(e))
    {
        return Ok(());
    }

    // Bubble up the remaining errors
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
            // todo: this should wait for based on id for some amount of time
            device.write_status(device_id, 0, 3, |buffer| {
                buffer[..2].copy_from_slice(&MODEL_NUMBER.to_le_bytes()); // u16 MODEL NUMBER
                buffer[2] = FIRMWARE_VERSION; //u8 FIRMWARE VERSION
                Ok(())
            })?;
            info!("Ping");
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
            registers[BUTTON_START_REG..BUTTON_START_REG + NUM_BUTTONS]
                .copy_from_slice(&buttons.read_u8());

            // Answer the read request
            device.write_status(device_id, 0, length, |buffer| {
                buffer.copy_from_slice(&registers[address..address + length]);
                Ok(())
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
