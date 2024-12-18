#include <array>
#include <Preferences.h>
#include <Dynamixel2Arduino.h>
#include <FastLED.h>
#include "BMI088.h"
#include "complementary_filter.h"
#include "uart_port_handler.h"
#include "utilities.h"

#define DEBUG false

// Pin definitions
#define DXL_DIR_PIN 22
#define DXL_RX_PIN 21
#define DXL_TX_PIN 23

#define ACCEL_CS 26
#define GYRO_CS 18

#define SPI_MOSI 5
#define SPI_MISO 17
#define SPI_SCK 19
#define INT_ACCEL 25
#define INT_GYRO 16

#define LED_PIN 27
#define BUTTON0_PIN 2
#define BUTTON1_PIN 32
#define BUTTON2_PIN 4

// UART definitions
#define DXL_UART UART_NUM_0
#define DEBUG_SERIAL Serial1

// Dynamixel definitions
#define DXL_PROTOCOL_VER_2_0 2.0
#define DXL_MODEL_NUM 0xbaff
#define DEFAULT_ID 241
#define DEFAULT_BAUD 3 // 1mbaud

#define ADDR_CONTROL_ITEM_BAUD 8

#define ADDR_CONTROL_ITEM_LED0_R 10
#define ADDR_CONTROL_ITEM_LED0_G 11
#define ADDR_CONTROL_ITEM_LED0_B 12
#define ADDR_CONTROL_ITEM_LED1_R 14
#define ADDR_CONTROL_ITEM_LED1_G 15
#define ADDR_CONTROL_ITEM_LED1_B 16
#define ADDR_CONTROL_ITEM_LED2_R 18
#define ADDR_CONTROL_ITEM_LED2_G 19
#define ADDR_CONTROL_ITEM_LED2_B 20

#define ADDR_CONTROL_ITEM_GYRO_X 36
#define ADDR_CONTROL_ITEM_GYRO_Y 40
#define ADDR_CONTROL_ITEM_GYRO_Z 44
#define ADDR_CONTROL_ITEM_ACCEL_X 48
#define ADDR_CONTROL_ITEM_ACCEL_Y 52
#define ADDR_CONTROL_ITEM_ACCEL_Z 56
#define ADDR_CONTROL_ITEM_QUAT_X 60
#define ADDR_CONTROL_ITEM_QUAT_Y 64
#define ADDR_CONTROL_ITEM_QUAT_Z 68
#define ADDR_CONTROL_ITEM_QUAT_W 72

#define ADDR_CONTROL_ITEM_BUTTON0 76
#define ADDR_CONTROL_ITEM_BUTTON1 77
#define ADDR_CONTROL_ITEM_BUTTON2 78

#define ADDR_CONTROL_ITEM_GYRO_ODR 100
#define ADDR_CONTROL_ITEM_ACCEL_ODR 101

#define ADDR_CONTROL_ITEM_GYRO_RANGE 102
#define ADDR_CONTROL_ITEM_ACCEL_RANGE 103

#define ADDR_CONTROL_ITEM_DO_ADAPTIVE_GAIN 108
#define ADDR_CONTROL_ITEM_DO_BIAS_ESTIMATION 109
#define ADDR_CONTROL_ITEM_ACCEL_GAIN 110
#define ADDR_CONTROL_ITEM_BIAS_ALPHA 114


// default parameters for BMI088
#define ACCEL_RANGE_DEFAULT  1 // Bmi088Accel::RANGE_6G
#define ACCEL_ODR_DEFAULT   15 // Bmi088Accel::ODR_400HZ_BW_40HZ
#define GYRO_RANGE_DEFAULT   3 // Bmi088Gyro::RANGE_1000DPS
#define GYRO_ODR_DEFAULT     5 // Bmi088Gyro::ODR_1000HZ_BW_116HZ

// default parameters for complementary filter
#define IMU_GAIN_ACCEL_DEFAULT 0.04
#define IMU_DO_ADAPTIVE_GAIN_DEFAULT false
#define IMU_DO_BIAS_ESTIMATION_DEFAULT false // Not working as we are not reading synchroneously
#define IMU_BIAS_ALPHA_DEFAULT 0.01

#define NUM_LEDS 3

// function definitions
void task_dxl(void *pvParameters);
void task_imu(void *pvParameters);

void write_callback_func(uint16_t item_addr, uint8_t &dxl_err_code, void *arg);

void setAccelRange(uint8_t range);
void setGyroRange(uint8_t range);