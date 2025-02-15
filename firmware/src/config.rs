use core::cell::RefCell;

use critical_section::Mutex;
use embedded_storage::{ReadStorage, Storage};
use esp_hal::delay;
use esp_storage::FlashStorage;
use heapless::{String, Vec};
use log::{info, warn};
use serde::{Deserialize, Serialize};
use serde_json_core;

// 128 bytes are used to store the config as a json in flash memory
// This is not the most efficient way to store the data, but we have enough space
// and it solves a lot of problems with serialization and deserialization of old
// data when the struct changes or the microcontroller is initially flashed
const CONFIG_SIZE: usize = 128;

#[derive(Serialize, Deserialize, Debug)]
pub struct Config {
    pub bus_boudrate: u32,
    pub id: u8,
    pub timeout: u64,
}

pub struct ConfigManager {
    config: Mutex<RefCell<Config>>,
    flash: Mutex<RefCell<FlashStorage>>,
    flash_offset: u32,
}

impl Config {
    fn default() -> Self {
        Self {
            bus_boudrate: 115200,
            id: 1,
            timeout: 0,
        }
    }
}

impl ConfigManager {
    pub fn new(flash: FlashStorage, flash_offset: u32) -> Self {
        let config_manager = Self {
            config: Mutex::new(RefCell::new(Config::default())),
            flash: Mutex::new(RefCell::new(flash)),
            flash_offset,
        };
        if config_manager.load_from_flash().is_err() {
            warn!("Failed to load config from flash, using default values");
            config_manager.commit();
        }
        config_manager
    }

    // Commit the current config to flash
    // TODO this currently sometimes freezes the device
    // The reason is unknown, but but might be linked to
    // https://github.com/esp-rs/esp-hal/issues/1714
    fn commit(&self) {
        // Create a buffer with spaces
        let mut buffer = [b' '; CONFIG_SIZE];

        critical_section::with(|cs| {
            // Serialize the config into a json string
            let serialized =
                serde_json_core::to_string::<Config, CONFIG_SIZE>(&*self.config.borrow_ref(cs))
                    .expect("Failed to serialize config");
            // Add string to padded buffer
            buffer[..serialized.len()].copy_from_slice(serialized.as_bytes());
        });

        critical_section::with(|cs| {
            // Write the json to flash
            self.flash
                .borrow_ref_mut(cs)
                .write(self.flash_offset, &buffer)
                .expect("Failed to write data to flash");
        });
    }

    fn load_from_flash(&self) -> Result<(), ()> {
        let mut buffer = [0u8; CONFIG_SIZE];

        critical_section::with(|cs| {
            self.flash
                .borrow_ref_mut(cs)
                .read(self.flash_offset, &mut buffer)
                .expect("Failed to read data from flash");
        });

        info!("Config data: {:?}", buffer);

        // Try to parse the data as json and deserialize it
        if let Ok(data) = String::<CONFIG_SIZE>::from_utf8(Vec::from_slice(&buffer).unwrap()) {
            // Log the config data
            info!("Config data: {}", data);
            if let Ok(config) = serde_json_core::from_str::<Config>(&data) {
                critical_section::with(|cs| {
                    *self.config.borrow_ref_mut(cs) = config.0;
                });
                return Ok(());
            }
        }
        Err(())
    }

    #[allow(dead_code)]
    pub fn wipe(&self) {
        critical_section::with(|cs| {
            self.flash
                .borrow_ref_mut(cs)
                .write(self.flash_offset, &[0; CONFIG_SIZE])
                .expect("Failed to erase flash");
        });
    }

    pub fn get<T>(&self, f: impl FnOnce(&Config) -> T) -> T {
        critical_section::with(|cs| f(&self.config.borrow_ref(cs)))
    }

    pub fn set<T>(&self, f: impl FnOnce(&mut Config) -> T) -> T {
        info!("Setting config in ram");
        let v = critical_section::with(|cs| f(&mut self.config.borrow_ref_mut(cs)));
        info!("Committing config to flash");
        self.commit();
        v
    }
}
