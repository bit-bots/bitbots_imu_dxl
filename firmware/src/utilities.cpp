#include "utilities.h"

uint32_t dxl_to_real_baud(uint8_t baud)
{
  int real_baud = 57600;
  switch(baud)
  {
    case 0: real_baud = 9600; break;
    case 1: real_baud = 57600; break;
    case 2: real_baud = 115200; break;
    case 3: real_baud = 1000000; break;
    case 4: real_baud = 2000000; break;
    case 5: real_baud = 3000000; break;
    case 6: real_baud = 4000000; break;
    case 7: real_baud = 4500000; break;
  }
  return real_baud;
}

void setAccelRange(Bmi088Accel &accel_handle, uint8_t range)
{
  Bmi088Accel::Range accel_range;
  switch (range)
  {
  case 0:
    accel_range = Bmi088Accel::RANGE_3G;
    break;
  case 1:
    accel_range = Bmi088Accel::RANGE_6G;
    break;
  case 2:
    accel_range = Bmi088Accel::RANGE_12G;
    break;
  case 3:
    accel_range = Bmi088Accel::RANGE_24G;
    break;
  default:
    accel_range = Bmi088Accel::RANGE_6G;
  }
  accel_handle.setRange(accel_range);
}

void setGyroRange(Bmi088Gyro &gyro_handle, uint8_t range)
{
  Bmi088Gyro::Range gyro_range;
  switch (range)
  {
  case 0:
    gyro_range = Bmi088Gyro::RANGE_125DPS;
    break;
  case 1:
    gyro_range = Bmi088Gyro::RANGE_250DPS;
    break;
  case 2:
    gyro_range = Bmi088Gyro::RANGE_500DPS;
    break;
  case 3:
    gyro_range = Bmi088Gyro::RANGE_1000DPS;
    break;
  case 4:
    gyro_range = Bmi088Gyro::RANGE_2000DPS;
    break;
  default:
    gyro_range = Bmi088Gyro::RANGE_1000DPS;
  }
  gyro_handle.setRange(gyro_range);
}

void setAccelOdr(Bmi088Accel &accel_handle, uint8_t odr)
{
  Bmi088Accel::Odr accel_odr;
  switch (odr)
  {
  case 0:
    accel_odr = Bmi088Accel::ODR_12_5HZ_BW_1HZ;
    break;
  case 1:
    accel_odr = Bmi088Accel::ODR_12_5HZ_BW_2HZ;
    break;
  case 2:
    accel_odr = Bmi088Accel::ODR_12_5HZ_BW_5HZ;
    break;
  case 3:
    accel_odr = Bmi088Accel::ODR_25HZ_BW_3HZ;
    break;
  case 4:
    accel_odr = Bmi088Accel::ODR_25HZ_BW_5HZ;
    break;
  case 5:
    accel_odr = Bmi088Accel::ODR_25HZ_BW_10HZ;
    break;
  case 6:
    accel_odr = Bmi088Accel::ODR_50HZ_BW_5HZ;
    break;
  case 7:
    accel_odr = Bmi088Accel::ODR_50HZ_BW_9HZ;
    break;
  case 8:
    accel_odr = Bmi088Accel::ODR_50HZ_BW_20HZ;
    break;
  case 9:
    accel_odr = Bmi088Accel::ODR_100HZ_BW_10HZ;
    break;
  case 10:
    accel_odr = Bmi088Accel::ODR_100HZ_BW_19HZ;
    break;
  case 11:
    accel_odr = Bmi088Accel::ODR_100HZ_BW_40HZ;
    break;
  case 12:
    accel_odr = Bmi088Accel::ODR_200HZ_BW_20HZ;
    break;
  case 13:
    accel_odr = Bmi088Accel::ODR_200HZ_BW_38HZ;
    break;
  case 14:
    accel_odr = Bmi088Accel::ODR_200HZ_BW_80HZ;
    break;
  case 15:
    accel_odr = Bmi088Accel::ODR_400HZ_BW_40HZ;
    break;
  case 16:
    accel_odr = Bmi088Accel::ODR_400HZ_BW_75HZ;
    break;
  case 17:
    accel_odr = Bmi088Accel::ODR_400HZ_BW_145HZ;
    break;
  case 18:
    accel_odr = Bmi088Accel::ODR_800HZ_BW_80HZ;
    break;
  case 19:
    accel_odr = Bmi088Accel::ODR_800HZ_BW_140HZ;
    break;
  case 20:
    accel_odr = Bmi088Accel::ODR_800HZ_BW_230HZ;
    break;
  case 21:
    accel_odr = Bmi088Accel::ODR_1600HZ_BW_145HZ;
    break;
  case 22:
    accel_odr = Bmi088Accel::ODR_1600HZ_BW_234HZ;
    break;
  case 23:
    accel_odr = Bmi088Accel::ODR_1600HZ_BW_280HZ;
    break;
  default:
    accel_odr = Bmi088Accel::ODR_400HZ_BW_40HZ;
  }
  accel_handle.setOdr(accel_odr);
}

void setGyroOdr(Bmi088Gyro &gyro_handle, uint8_t odr)
{
  Serial1.println("Setting gyro ODR");
  Bmi088Gyro::Odr gyro_odr;
  switch (odr)
  {
  case 0:
    gyro_odr = Bmi088Gyro::ODR_100HZ_BW_12HZ;
    break;
  case 1:
    gyro_odr = Bmi088Gyro::ODR_100HZ_BW_32HZ;
    break;
  case 2:
    gyro_odr = Bmi088Gyro::ODR_200HZ_BW_64HZ;
    break;
  case 3:
    gyro_odr = Bmi088Gyro::ODR_200HZ_BW_23HZ;
    break;
  case 4:
    gyro_odr = Bmi088Gyro::ODR_400HZ_BW_47HZ;
    break;
  case 5:
    gyro_odr = Bmi088Gyro::ODR_1000HZ_BW_116HZ;
    break;
  case 6:
    gyro_odr = Bmi088Gyro::ODR_2000HZ_BW_230HZ;
    break;
  case 7:
    gyro_odr = Bmi088Gyro::ODR_2000HZ_BW_532HZ;
    break;
  default:
    gyro_odr = Bmi088Gyro::ODR_1000HZ_BW_116HZ;
  }
  gyro_handle.setOdr(gyro_odr);
}