use core::{cell::RefCell, fmt, time::Duration};

use bmi088::{Accelerometer, Gyroscope};
use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{delay::MicrosDurationU64, time::now};
use imu_fusion::FusionAhrsSettings;
use imu_fusion::{FusionQuaternion, FusionVector};
use log::error;
#[cfg(feature = "profiling")]
use log::info;

#[derive(Clone, Copy)]
pub struct IMUState {
    pub orientation: FusionQuaternion,
    pub gyro: FusionVector,
    pub accel: FusionVector,
}

impl IMUState {
    pub fn default() -> Self {
        Self {
            orientation: FusionQuaternion::identity(),
            gyro: FusionVector::zero(),
            accel: FusionVector::zero(),
        }
    }

    pub fn to_le_buffer(self) -> [u8; 40] {
        let mut buffer = [0; 40];
        let elements = [
            self.gyro.x,
            self.gyro.y,
            self.gyro.z,
            self.accel.x,
            self.accel.y,
            self.accel.z,
            self.orientation.x,
            self.orientation.y,
            self.orientation.z,
            self.orientation.w,
        ];
        for (i, element) in elements.iter().enumerate() {
            let bytes = element.to_le_bytes();
            buffer[i * bytes.len()..(i + 1) * bytes.len()].copy_from_slice(&bytes);
        }
        buffer
    }
}

impl fmt::Debug for IMUState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("IMUState")
            .field(
                "orientation",
                &format_args!(
                    "{:.2}, {:.2}, {:.2}, {:.2}",
                    self.orientation.w, self.orientation.x, self.orientation.y, self.orientation.z
                ),
            )
            .field(
                "gyro",
                &format_args!("{:.2}, {:.2}, {:.2}", self.gyro.x, self.gyro.y, self.gyro.z),
            )
            .field(
                "accel",
                &format_args!(
                    "{:.2}, {:.2}, {:.2}",
                    self.accel.x, self.accel.y, self.accel.z
                ),
            )
            .finish()
    }
}

pub fn filter_loop<S>(
    imu_state: &Mutex<RefCell<IMUState>>,
    mut gyro: Gyroscope<bmi088::SpiInterface<S>>,
    mut accel: Accelerometer<bmi088::SpiInterface<S>>,
) -> !
where
    S: embedded_hal::spi::SpiDevice + embedded_hal::spi::ErrorType,
{
    // Setup the Sensor Fusion
    let mut ahrs_settings = FusionAhrsSettings::new();
    ahrs_settings.gain = 0.05f32; // Default is 0.5, but we can make it more aggressive because our IMU is very good and we want little noise due to a noisy gravity vector
    let mut fusion = imu_fusion::Fusion::new(crate::IMU_SAMPLE_RATE_HZ, ahrs_settings);

    // Timing stuff
    let mut previous_time = now();

    // Profiling
    #[cfg(feature = "profiling")]
    let mut counter = 0;
    #[cfg(feature = "profiling")]
    let mut start_time = now();

    // Main sensor loop
    loop {
        // Get the start time
        let cycle_begin = now();

        // Get the latest Gyroscope data
        let gyro_sample = match gyro.get_gyro() {
            Ok(sample) => imu_fusion::FusionVector::new(
                // Cast to f32 and scale to degrees per second
                sample[1] as f32 / i16::MAX as f32 * crate::GYRO_RANGE,
                sample[0] as f32 / i16::MAX as f32 * crate::GYRO_RANGE,
                -sample[2] as f32 / i16::MAX as f32 * crate::GYRO_RANGE,
            ),
            Err(e) => {
                error!("Failed to get gyro data: {:?}", e);
                continue;
            }
        };

        // Get the latest Accelerometer data
        let accel_sample = match accel.get_accel() {
            Ok(sample) => imu_fusion::FusionVector::new(
                // Cast to f32 and scale to m/s^2
                sample[1] as f32 / i16::MAX as f32 * crate::ACCEL_RANGE * 9.81,
                sample[0] as f32 / i16::MAX as f32 * crate::ACCEL_RANGE * 9.81,
                -sample[2] as f32 / i16::MAX as f32 * crate::ACCEL_RANGE * 9.81,
            ),
            Err(e) => {
                error!("Failed to get accel data: {:?}", e);
                continue;
            }
        };

        // Time keeping
        let current_time = now();
        let delta_time = current_time - previous_time;

        // Update the filter
        fusion.update_no_mag_by_duration_seconds(
            gyro_sample,
            accel_sample,
            Duration::from_nanos(delta_time.to_nanos()).as_secs_f32(),
        );

        // Update the shared state
        critical_section::with(|cs| {
            imu_state.borrow(cs).replace(IMUState {
                orientation: fusion.quaternion(),
                gyro: gyro_sample,
                accel: accel_sample,
            });
        });

        // Update the time
        previous_time = current_time;

        // Delay to keep the loop rate (busy wait because the delay is not accurate enough)
        let mut cycle_time = now() - cycle_begin;
        while cycle_time < MicrosDurationU64::Hz(crate::IMU_SAMPLE_RATE_HZ as u64) {
            cycle_time = now() - cycle_begin;
        }

        // Profiling
        #[cfg(feature = "profiling")]
        {
            counter += 1;
            if counter % 1000 == 0 {
                info!(
                    "Loop rate: {}",
                    1000.0 / Duration::from_nanos((now() - start_time).to_nanos()).as_secs_f32()
                );
                start_time = now();
            }
        }
    }
}
