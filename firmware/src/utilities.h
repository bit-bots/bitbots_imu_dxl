#include <Arduino.h>
#include "BMI088.h"

uint32_t dxl_to_real_baud(uint8_t baud);
void setAccelRange(Bmi088Accel &accel_handle, uint8_t range);
void setGyroRange(Bmi088Gyro &gyro_handle, uint8_t range);
void setAccelOdr(Bmi088Accel &accel_handle, uint8_t odr);
void setGyroOdr(Bmi088Gyro &gyro_handle, uint8_t odr);
