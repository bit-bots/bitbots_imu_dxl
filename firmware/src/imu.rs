use core::fmt;
use imu_fusion::{FusionQuaternion, FusionVector};

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

    pub fn to_le_buffer(&self) -> [u8; 40] {
        let mut buffer = [0; 40];
        let elements = [
            self.orientation.w,
            self.orientation.x,
            self.orientation.y,
            self.orientation.z,
            self.gyro.x,
            self.gyro.y,
            self.gyro.z,
            self.accel.x,
            self.accel.y,
            self.accel.z,
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
