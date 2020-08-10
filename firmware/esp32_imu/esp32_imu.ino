#include <MPU9250.h> // modified version required since we use mpu6500
#include <Dynamixel2Arduino.h>
#include <FastLED.h>
#include <array>
#include "complementary_filter.h"
#include <Preferences.h>
#include "fast_slave.h"

// define two tasks for reading the dxl bus and doing other work
void TaskDXL( void *pvParameters );
void TaskWorker( void *pvParameters );

TaskHandle_t th_dxl,th_worker;

Preferences dxl_prefs;
Preferences imu_prefs;

/*---------------------- DXL defines and variables ---------------------*/

#define DXL_DIR_PIN 22
#define DXL_PROTOCOL_VER_2_0 2.0
#define DXL_MODEL_NUM 0xbaff
#define DEFAULT_ID 241
#define DEFAULT_BAUD 4 //2mbaud
DYNAMIXEL::SerialPortHandler dxl_port(Serial, DXL_DIR_PIN);
DYNAMIXEL::Slave dxl(dxl_port, DXL_MODEL_NUM);

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

#define ADDR_CONTROL_ITEM_GYRO_RANGE 102
#define ADDR_CONTROL_ITEM_ACCEL_RANGE 103
#define ADDR_CONTROL_ITEM_CALIBRATE_GYRO 104
#define ADDR_CONTROL_ITEM_RESET_GYRO_CALIBRATION 105
#define ADDR_CONTROL_ITEM_CALIBRATE_ACCEL 104

#define ADDR_CONTROL_ITEM_DO_ADAPTIVE_GAIN 108
#define ADDR_CONTROL_ITEM_DO_BIAS_ESTIMATION 109
#define ADDR_CONTROL_ITEM_ACCEL_GAIN 110
#define ADDR_CONTROL_ITEM_BIAS_ALPHA 114



// id and baud are stored using preferences for persistance between resets of the chip
uint8_t id;
uint8_t baud;


/*---------------------- IMU defines and variables ---------------------*/

#define IMU_NCS 5
#define IMU_INTERRUPT 17

MPU9250 IMU(SPI, IMU_NCS);

#define ACCEL_RANGE_DEFAULT MPU9250::ACCEL_RANGE_16G
#define GYRO_RANGE_DEFAULT MPU9250::GYRO_RANGE_2000DPS
#define SAMPLE_RATE_DIVIDER 0  // (freq = 1000 / (1+srd))


float gyro[3] = {0,0,0};
float accel[3] = {0,0,0};
float quat[4] = {0,0,0,0};

uint8_t gyro_range, accel_range;

#define IMU_GAIN_ACCEL_DEFAULT 0.04
#define IMU_DO_ADAPTIVE_GAIN_DEFAULT false
#define IMU_DO_BIAS_ESTIMATION_DEFAULT true
#define IMU_BIAS_ALPHA_DEFAULT 0.01

bool calibrate_gyro = false;
bool reset_gyro_calibration = false;
bool do_adaptive_gain, do_bias_estimation;
float accel_gain, bias_alpha;

imu_tools::ComplementaryFilter filter_;
float dt = 0; // time delta between readings

/*---------------------- LEDS ---------------------*/
#define LED_PIN 4
#define NUM_LEDS 3
CRGB leds[NUM_LEDS];

/*---------------------- Buttons ---------------------*/
#define BUTTON0_PIN 27
#define BUTTON1_PIN 14
#define BUTTON2_PIN 12

uint8_t buttons[3];

void setup() {
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  leds[0] = CRGB::Red;
  leds[1] = CRGB::Blue;
  leds[2] = CRGB::Green;
  FastLED.show();
  delay(200);
  leds[0] = CRGB::Black;
  leds[1] = CRGB::Black;
  leds[2] = CRGB::Black;
  FastLED.show();
  FastLED.setBrightness(128);
  
  disableCore0WDT(); // required since we dont want FreeRTOS to slow down our reading if the Wachdogtimer (WTD) fires
  disableCore1WDT();
  xTaskCreatePinnedToCore(
    TaskDXL
    ,  "TaskDXL"   // A name just for humans
    ,  65536  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority 3 since otherwise 
    ,  &th_dxl 
    ,  0);

  xTaskCreatePinnedToCore(
    TaskWorker
    ,  "TaskWork"
    ,  65536  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  &th_worker 
    ,  1);
}

void loop()
{
  // see tasks :D
}


/*---------------------- DXL ---------------------*/

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
    case 7: real_baud = 5000000; break;
  }
  return real_baud;
}

void write_callback_func(uint16_t item_addr, uint8_t &dxl_err_code, void* arg)
{
  (void)dxl_err_code, (void)arg;
  if (item_addr == ADDR_CONTROL_ITEM_BAUD)
  {
    dxl_prefs.putUChar("baud", baud);
    ESP.restart(); // restart whole chip since restarting serial port crashes esp
  } 
  else if(item_addr == ADDR_CONTROL_ITEM_GYRO_RANGE)
  {
    setGyroRange(gyro_range);
    imu_prefs.putUChar("gyro_range", gyro_range);

  }
  else if(item_addr == ADDR_CONTROL_ITEM_ACCEL_RANGE)
  {
    setAccelRange(accel_range);
    imu_prefs.putUChar("accel_range", accel_range);
  }
  else if(item_addr == ADDR_CONTROL_ITEM_CALIBRATE_GYRO)
  {
    if(calibrate_gyro)
    {
      IMU.calibrateGyro();
      FastLED.show();
      calibrate_gyro = false;
    }
  }
  else if(item_addr == ADDR_CONTROL_ITEM_RESET_GYRO_CALIBRATION)
  {
    if(reset_gyro_calibration)
    {
      IMU.setGyroBiasX_rads(0);
      IMU.setGyroBiasY_rads(0);
      IMU.setGyroBiasZ_rads(0);
      reset_gyro_calibration = 0;
    }
  }
  else if(item_addr == ADDR_CONTROL_ITEM_DO_ADAPTIVE_GAIN)
  {
    imu_prefs.putUChar("adaptive_gain", do_adaptive_gain);
    filter_.setDoAdaptiveGain(do_adaptive_gain);
  }
  else if(item_addr == ADDR_CONTROL_ITEM_DO_BIAS_ESTIMATION)
  {
    imu_prefs.putUChar("bias_estimation", do_bias_estimation);
    filter_.setDoBiasEstimation(do_bias_estimation);
  }
  else if(item_addr == ADDR_CONTROL_ITEM_ACCEL_GAIN)
  {
    imu_prefs.putFloat("gain_accel", IMU_GAIN_ACCEL_DEFAULT);
    filter_.setGainAcc(accel_gain);
  }
  else if(item_addr == ADDR_CONTROL_ITEM_BIAS_ALPHA)
  {
    imu_prefs.putFloat("bias_alpha", bias_alpha);
    filter_.setBiasAlpha(bias_alpha);
  }
  else if(item_addr >= ADDR_CONTROL_ITEM_LED0_R && item_addr <= ADDR_CONTROL_ITEM_LED2_B)
  {
    FastLED.show();
  }
}

void TaskDXL(void *pvParameters) 
{
  (void) pvParameters;

  dxl_prefs.begin("dxl");
  if(!dxl_prefs.getUChar("init")) // check if prefs are initialized
  {
    dxl_prefs.putUChar("id", DEFAULT_ID);
    dxl_prefs.putUChar("baud", DEFAULT_BAUD);
    dxl_prefs.putUChar("init",1); // set initialized
  }
  id = dxl_prefs.getUChar("id");
  baud = dxl_prefs.getUChar("baud");

  dxl_port.begin(dxl_to_real_baud(baud));

  dxl.setPortProtocolVersion(DXL_PROTOCOL_VER_2_0);
  dxl.setFirmwareVersion(1);
  dxl.setID(id);
  
  //dxl.addControlItem(ADDR_CONTROL_ITEM_ID, id); // not allowed since already implemented in library as default item
  dxl.addControlItem(ADDR_CONTROL_ITEM_BAUD, baud);

  dxl.addControlItem(ADDR_CONTROL_ITEM_LED0_R, leds[0].r);
  dxl.addControlItem(ADDR_CONTROL_ITEM_LED0_G, leds[0].g);
  dxl.addControlItem(ADDR_CONTROL_ITEM_LED0_B, leds[0].b);
  dxl.addControlItem(ADDR_CONTROL_ITEM_LED1_R, leds[1].r);
  dxl.addControlItem(ADDR_CONTROL_ITEM_LED1_G, leds[1].g);
  dxl.addControlItem(ADDR_CONTROL_ITEM_LED1_B, leds[1].b);
  dxl.addControlItem(ADDR_CONTROL_ITEM_LED2_R, leds[2].r);
  dxl.addControlItem(ADDR_CONTROL_ITEM_LED2_G, leds[2].g);
  dxl.addControlItem(ADDR_CONTROL_ITEM_LED2_B, leds[2].b);

  dxl.addControlItem(ADDR_CONTROL_ITEM_GYRO_X, gyro[0]);
  dxl.addControlItem(ADDR_CONTROL_ITEM_GYRO_Y, gyro[1]);
  dxl.addControlItem(ADDR_CONTROL_ITEM_GYRO_Z, gyro[2]);
  dxl.addControlItem(ADDR_CONTROL_ITEM_ACCEL_X, accel[0]);
  dxl.addControlItem(ADDR_CONTROL_ITEM_ACCEL_Y, accel[1]);
  dxl.addControlItem(ADDR_CONTROL_ITEM_ACCEL_Z, accel[2]);
  dxl.addControlItem(ADDR_CONTROL_ITEM_QUAT_X, quat[0]);
  dxl.addControlItem(ADDR_CONTROL_ITEM_QUAT_Y, quat[1]);
  dxl.addControlItem(ADDR_CONTROL_ITEM_QUAT_Z, quat[2]);
  dxl.addControlItem(ADDR_CONTROL_ITEM_QUAT_W, quat[3]);
  
  dxl.addControlItem(ADDR_CONTROL_ITEM_BUTTON0, buttons[0]);
  dxl.addControlItem(ADDR_CONTROL_ITEM_BUTTON1, buttons[1]);
  dxl.addControlItem(ADDR_CONTROL_ITEM_BUTTON2, buttons[2]);
  
  dxl.addControlItem(ADDR_CONTROL_ITEM_GYRO_RANGE, gyro_range);
  dxl.addControlItem(ADDR_CONTROL_ITEM_ACCEL_RANGE, accel_range);

  
  dxl.addControlItem(ADDR_CONTROL_ITEM_CALIBRATE_GYRO, calibrate_gyro);
  dxl.addControlItem(ADDR_CONTROL_ITEM_RESET_GYRO_CALIBRATION, reset_gyro_calibration);

  dxl.addControlItem(ADDR_CONTROL_ITEM_DO_ADAPTIVE_GAIN, do_adaptive_gain);
  dxl.addControlItem(ADDR_CONTROL_ITEM_DO_BIAS_ESTIMATION, do_bias_estimation);
  dxl.addControlItem(ADDR_CONTROL_ITEM_ACCEL_GAIN, accel_gain);
  dxl.addControlItem(ADDR_CONTROL_ITEM_BIAS_ALPHA, bias_alpha);

  
  dxl.setWriteCallbackFunc(write_callback_func);
  /*
  // init uart 0, given baud, 8bits 1stop no parity, pin 3 and 1, 256 buffer, no inversion
  uart = uartBegin(0, dxl_to_real_baud(baud), SERIAL_8N1, 3, 1,  256, false);
  // disable all interrupts
  uart->dev->conf1.rx_tout_en = 0;
  uart->dev->int_ena.rxfifo_full = 0;
  uart->dev->int_ena.frm_err = 0;
  uart->dev->int_ena.rxfifo_tout = 0;
  */
  for (;;)
  {
    if(dxl.processPacket()){
      if(dxl.getID() != id) // since we cant add the id as a control item, we need to check if it has been updated manually
      {
        id = dxl.getID();
        dxl_prefs.putUChar("id", id);
      }
    }
  }
}

/*---------------------- IMU ---------------------*/
void setAccelRange(uint8_t range)
{
    MPU9250::AccelRange mpu_range;
    switch(range)
    {
      case 0:
        mpu_range = MPU9250::ACCEL_RANGE_2G;
        break;
      case 1:
        mpu_range = MPU9250::ACCEL_RANGE_4G;
        break;
      case 2:
        mpu_range = MPU9250::ACCEL_RANGE_8G;
        break;
      case 3:
        mpu_range = MPU9250::ACCEL_RANGE_16G;
        break;
      default:
        mpu_range = ACCEL_RANGE_DEFAULT;
    }
    IMU.setAccelRange(mpu_range);
}
void setGyroRange(uint8_t range)
{
    MPU9250::GyroRange mpu_range;
    switch(range)
    {
      case 0:
        mpu_range = MPU9250::GYRO_RANGE_250DPS;
        break;
      case 1:
        mpu_range = MPU9250::GYRO_RANGE_500DPS;
        break;
      case 2:
        mpu_range = MPU9250::GYRO_RANGE_1000DPS;
        break;
      case 3:
        mpu_range = MPU9250::GYRO_RANGE_2000DPS;
        break;
      default:
        mpu_range = GYRO_RANGE_DEFAULT;
    }
    IMU.setGyroRange(mpu_range);
}
void getIMU(){
    if(calibrate_gyro)
      return;
    
    buttons[0] = !digitalRead(BUTTON0_PIN);
    buttons[1] = !digitalRead(BUTTON1_PIN);
    buttons[2] = !digitalRead(BUTTON2_PIN);
    
    IMU.readSensor();
    dt += 1e-3;
    float tmp_gyro[3], tmp_accel[3];
    tmp_gyro[0] = IMU.getGyroY_rads();
    tmp_gyro[1] = IMU.getGyroX_rads();
    tmp_gyro[2] = IMU.getGyroZ_rads();
    tmp_accel[0] = IMU.getAccelX_mss();
    tmp_accel[1] = IMU.getAccelY_mss();
    tmp_accel[2] = IMU.getAccelZ_mss();
    
    if(isnan(tmp_gyro[0]) || isnan(tmp_gyro[1]) || isnan(tmp_gyro[2]) || isnan(tmp_accel[0]) || isnan(tmp_accel[1]) || isnan(tmp_accel[2]))
      return;
    
    for(int i = 0; i<3; i++)
    {
      gyro[i] = tmp_gyro[i];
      accel[i] = tmp_accel[i];  
    }
    filter_.update(accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], 1e-3);
    dt = 0;
    double q0,q1,q2,q3;
    filter_.getOrientation(q0, q1, q2, q3); //hamilton to ros quaternion
    quat[0] = q3;
    quat[1] = q0;
    quat[2] = q1;
    quat[3] = q2;
    
    if (filter_.getDoBiasEstimation())
    {
      gyro[0] -= filter_.getAngularVelocityBiasX();
      gyro[1] -= filter_.getAngularVelocityBiasY();
      gyro[2] -= filter_.getAngularVelocityBiasZ();
    }
}

void TaskWorker(void *pvParameters) 
{
  (void) pvParameters;

  imu_prefs.begin("imu");

  if(!imu_prefs.getUChar("init")) // check if prefs are initialized
  {
    imu_prefs.putUChar("accel_range", ACCEL_RANGE_DEFAULT);
    imu_prefs.putUChar("gyro_range", GYRO_RANGE_DEFAULT);
    
    imu_prefs.putUChar("adaptive_gain", IMU_DO_ADAPTIVE_GAIN_DEFAULT);
    imu_prefs.putUChar("bias_estimation", IMU_DO_BIAS_ESTIMATION_DEFAULT);
    imu_prefs.putFloat("gain_accel", IMU_GAIN_ACCEL_DEFAULT);
    imu_prefs.putFloat("bias_alpha", IMU_BIAS_ALPHA_DEFAULT);
    
    imu_prefs.putUChar("init",1); // set initialized
  }
  delay(1000);
  int status = IMU.begin();
  Serial.print("status: ");
  Serial.println(status);
  IMU.setSrd(50);
  setGyroRange(imu_prefs.getUChar("gyro_range"));
  setAccelRange(imu_prefs.getUChar("accel_range"));
  IMU.enableDataReadyInterrupt();

  do_adaptive_gain = imu_prefs.getUChar("adaptive_gain");
  accel_gain = imu_prefs.getFloat("gain_accel");
  do_bias_estimation = imu_prefs.getUChar("do_bias_estimation");
  bias_alpha = imu_prefs.getFloat("bias_alpha");
  
  filter_.setDoAdaptiveGain(do_adaptive_gain);
  filter_.setGainAcc(accel_gain);
  filter_.setDoBiasEstimation(do_bias_estimation);
  filter_.setBiasAlpha(bias_alpha);

  pinMode(BUTTON0_PIN,INPUT_PULLUP);
  pinMode(BUTTON1_PIN,INPUT_PULLUP);
  pinMode(BUTTON2_PIN,INPUT_PULLUP);
  
  pinMode(IMU_INTERRUPT,INPUT);
  attachInterrupt(IMU_INTERRUPT,getIMU,RISING);

  for (;;)
  {
  }
  
}
