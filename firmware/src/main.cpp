#include <main.h>

TaskHandle_t th_dxl, th_worker;

Preferences dxl_prefs;
Preferences imu_prefs;

/*---------------------- DXL defines and variables ---------------------*/
ESP32UartPortHandler uart(DXL_UART, DXL_TX_PIN, DXL_RX_PIN, DXL_DIR_PIN);

DYNAMIXEL::Slave dxl(uart, DXL_MODEL_NUM, DXL_PROTOCOL_VER_2_0);

// id and baud are stored using preferences library for persistance between resets of the chip
uint8_t id;
uint8_t baud;

/*---------------------- IMU defines and variables ---------------------*/
Bmi088Accel accel_handle(SPI, ACCEL_CS);
Bmi088Gyro gyro_handle(SPI, GYRO_CS);

// flag triggered by interrupt to read data from IMU
volatile bool accel_drdy_flag, gyro_drdy_flag = false;

// lists for storing measurements and filter outputs
float gyro[3] = {0, 0, 0};
float accel[3] = {0, 0, 0};
float quat[4] = {0, 0, 0, 0};

// registers for parameters of IMU
uint8_t gyro_odr, accel_odr, gyro_range, accel_range;

// registers for parameters of complementary filter
bool do_adaptive_gain, do_bias_estimation;
float accel_gain, bias_alpha;

imu_tools::ComplementaryFilter filter_;
int64_t last_gyro_update = esp_timer_get_time();

// registers for leds and buttons
CRGB leds[NUM_LEDS];
uint8_t buttons[3];

uint8_t rx_timeout_thresh, txfifo_empty_thresh, rxfifo_full_thresh;

void setup()
{
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

#ifdef DEBUG
  DEBUG_SERIAL.begin(115200, SERIAL_8N1, 3, 1);
#endif

  disableCore0WDT(); // required since we dont want FreeRTOS to slow down our reading if the Wachdogtimer (WTD) fires
  disableCore1WDT();

  xTaskCreatePinnedToCore(task_dxl, "TaskDXL", 65536, NULL, 3, &th_dxl, 0);
  xTaskCreatePinnedToCore(task_imu, "TaskWork", 65536, NULL, 3, &th_worker, 1);
}

void loop()
{
  // see tasks :D
}

/*---------------------- DXL ---------------------*/
void task_dxl(void *pvParameters)
{
  (void)pvParameters;
  dxl_prefs.begin("dxl");
  if (dxl_prefs.getUChar("init") != 43) // check if prefs are initialized
  {
    dxl_prefs.putUChar("id", DEFAULT_ID);
    dxl_prefs.putUChar("baud", DEFAULT_BAUD);
    dxl_prefs.putUChar("rx_timeout_thresh", 1);
    dxl_prefs.putUChar("txfifo_empty_thresh", 1);
    dxl_prefs.putUChar("rxfifo_full_thresh", 1);
    dxl_prefs.putUChar("init", 42); // set initialized
  }

  id = dxl_prefs.getUChar("id");
  baud = dxl_prefs.getUChar("baud");

  rx_timeout_thresh = dxl_prefs.getUChar("rx_timeout_thresh");
  txfifo_empty_thresh = dxl_prefs.getUChar("txfifo_empty_thresh");
  rxfifo_full_thresh = dxl_prefs.getUChar("rxfifo_full_thresh");

  dxl.setPortProtocolVersion(DXL_PROTOCOL_VER_2_0);
  dxl.setFirmwareVersion(1);
  dxl.setID(id);

  // dxl.addControlItem(ADDR_CONTROL_ITEM_ID, id); // not allowed since already implemented in library as default item
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

  dxl.addControlItem(ADDR_CONTROL_ITEM_GYRO_X, gyro[0]); // TODO: 16 bit anstatt float
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

  dxl.addControlItem(ADDR_CONTROL_ITEM_GYRO_ODR, gyro_odr);
  dxl.addControlItem(ADDR_CONTROL_ITEM_ACCEL_ODR, accel_odr);
  dxl.addControlItem(ADDR_CONTROL_ITEM_GYRO_RANGE, gyro_range);
  dxl.addControlItem(ADDR_CONTROL_ITEM_ACCEL_RANGE, accel_range);

  dxl.addControlItem(ADDR_CONTROL_ITEM_DO_ADAPTIVE_GAIN, do_adaptive_gain);
  dxl.addControlItem(ADDR_CONTROL_ITEM_DO_BIAS_ESTIMATION, do_bias_estimation);
  dxl.addControlItem(ADDR_CONTROL_ITEM_ACCEL_GAIN, accel_gain);
  dxl.addControlItem(ADDR_CONTROL_ITEM_BIAS_ALPHA, bias_alpha);

  dxl.addControlItem(116, rx_timeout_thresh);
  dxl.addControlItem(117, txfifo_empty_thresh);
  dxl.addControlItem(118, rxfifo_full_thresh);

  dxl.setWriteCallbackFunc(write_callback_func);

  pinMode(DXL_DIR_PIN, OUTPUT);
  digitalWrite(DXL_DIR_PIN, LOW);

  uart.setBaudRate(dxl_to_real_baud(baud));
  uart.setItrParams(rx_timeout_thresh, txfifo_empty_thresh, rxfifo_full_thresh);
  uart.begin();

  for (;;)
  {
    if (dxl.processPacket())
    {
      if (dxl.getID() != id) // since we cant add the id as a control item, we need to check if it has been updated manually
      {
        id = dxl.getID();
        dxl_prefs.putUChar("id", id);
      }
    }
#ifdef DEBUG
    else
    {
      DEBUG_SERIAL.print("Last lib err code: ");
      DEBUG_SERIAL.print(dxl.getLastLibErrCode());
      DEBUG_SERIAL.print(", ");
      DEBUG_SERIAL.print("Last status packet err code: ");
      DEBUG_SERIAL.println(dxl.getLastStatusPacketError());
      DEBUG_SERIAL.println();
    }
#endif
  }
}

void write_callback_func(uint16_t item_addr, uint8_t &dxl_err_code, void *arg)
{
  (void)dxl_err_code, (void)arg;
  if (item_addr == ADDR_CONTROL_ITEM_BAUD)
  {
    dxl_prefs.putUChar("baud", baud);
    ESP.restart(); // restart whole chip since restarting it is easier
  }
  else if (item_addr == ADDR_CONTROL_ITEM_GYRO_ODR)
  {
    setGyroOdr(gyro_handle, gyro_odr);
    imu_prefs.putUChar("gyro_odr", gyro_odr);
  }
  else if (item_addr == ADDR_CONTROL_ITEM_ACCEL_ODR)
  {
    setAccelOdr(accel_handle, accel_odr);
    imu_prefs.putUChar("accel_odr", accel_odr);
  }
  else if (item_addr == ADDR_CONTROL_ITEM_GYRO_RANGE)
  {
    setGyroRange(gyro_handle, gyro_range);
    imu_prefs.putUChar("gyro_range", gyro_range);
  }
  else if (item_addr == ADDR_CONTROL_ITEM_ACCEL_RANGE)
  {
    setAccelRange(accel_handle, accel_range);
    imu_prefs.putUChar("accel_range", accel_range);
  }
  else if (item_addr == ADDR_CONTROL_ITEM_DO_ADAPTIVE_GAIN)
  {
    imu_prefs.putUChar("adaptive_gain", do_adaptive_gain);
    filter_.setDoAdaptiveGain(do_adaptive_gain);
  }
  else if (item_addr == ADDR_CONTROL_ITEM_DO_BIAS_ESTIMATION)
  {
    imu_prefs.putUChar("bias_estimation", do_bias_estimation);
    filter_.setDoBiasEstimation(do_bias_estimation);
  }
  else if (item_addr == ADDR_CONTROL_ITEM_ACCEL_GAIN)
  {
    imu_prefs.putFloat("gain_accel", accel_gain);
    filter_.setGainAcc(accel_gain);
  }
  else if (item_addr == ADDR_CONTROL_ITEM_BIAS_ALPHA)
  {
    imu_prefs.putFloat("bias_alpha", bias_alpha);
    filter_.setBiasAlpha(bias_alpha);
  }
  else if (item_addr >= ADDR_CONTROL_ITEM_LED0_R && item_addr <= ADDR_CONTROL_ITEM_LED2_B)
  {
    FastLED.show();
  }
  else if (item_addr == 116)
  {
    dxl_prefs.putUChar("rx_timeout_thresh", rx_timeout_thresh);
    //uart.setItrParams(rx_timeout_thresh, txfifo_empty_thresh, rxfifo_full_thresh);
    //uart.begin();
  }
  else if (item_addr == 117)
  {
    dxl_prefs.putUChar("txfifo_empty_thresh", txfifo_empty_thresh);
    //uart.setItrParams(rx_timeout_thresh, txfifo_empty_thresh, rxfifo_full_thresh);
    //uart.begin();
  }
  else if (item_addr == 118)
  {
    dxl_prefs.putUChar("rxfifo_full_thresh", rxfifo_full_thresh);
    //uart.setItrParams(rx_timeout_thresh, txfifo_empty_thresh, rxfifo_full_thresh);
    //uart.begin();
  
  }
}

/*---------------------- IMU ---------------------*/

void IRAM_ATTR accel_drdy_int() // IRAM_ATTR puts this function into ram, required since it is called as an interrupt
{
  accel_drdy_flag = true;
}

void IRAM_ATTR gyro_drdy_int() // IRAM_ATTR puts this function into ram, required since it is called as an interrupt
{
  gyro_drdy_flag = true;
}

void task_imu(void *pvParameters)
{


  // setup buttons
  pinMode(BUTTON0_PIN, INPUT_PULLUP);
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  buttons[0] = 0;
  buttons[1] = 0;
  buttons[2] = 0;

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  (void)pvParameters;
  imu_prefs.begin("imu");
  if (imu_prefs.getUChar("init_imu") != 42) // check if prefs are initialized
  {
    imu_prefs.putUChar("accel_odr", ACCEL_ODR_DEFAULT);
    imu_prefs.putUChar("gyro_odr", GYRO_ODR_DEFAULT);
    imu_prefs.putUChar("accel_range", ACCEL_RANGE_DEFAULT);
    imu_prefs.putUChar("gyro_range", GYRO_RANGE_DEFAULT);
    imu_prefs.putUChar("adaptive_gain", IMU_DO_ADAPTIVE_GAIN_DEFAULT);
    imu_prefs.putUChar("bias_estimation", IMU_DO_BIAS_ESTIMATION_DEFAULT);
    imu_prefs.putFloat("gain_accel", IMU_GAIN_ACCEL_DEFAULT);
    imu_prefs.putFloat("bias_alpha", IMU_BIAS_ALPHA_DEFAULT);
    imu_prefs.putUChar("init_imu", 42); // set initialized
  }

  // setup filter with defaults
  do_adaptive_gain = imu_prefs.getUChar("adaptive_gain");
  accel_gain = imu_prefs.getFloat("gain_accel");
  do_bias_estimation = imu_prefs.getUChar("do_bias_estimation");
  bias_alpha = imu_prefs.getFloat("bias_alpha");

  filter_.setDoAdaptiveGain(do_adaptive_gain);
  filter_.setGainAcc(accel_gain);
  filter_.setDoBiasEstimation(do_bias_estimation);
  filter_.setBiasAlpha(bias_alpha);


  int accel_status = accel_handle.begin();
  int gyro_status = gyro_handle.begin();

#ifdef DEBUG
  DEBUG_SERIAL.print("Accel status: ");
  DEBUG_SERIAL.println(accel_status);
  DEBUG_SERIAL.print("Gyro status: ");
  DEBUG_SERIAL.println(gyro_status);
#endif

  if (accel_status == 1 && gyro_status == 1)
  {
    leds[2] = CRGB::Green;
  }
  else if (accel_status != 1)
  {
    leds[1] = CRGB::Red;
  }
  else if (gyro_status != 1)
  {
    leds[2] = CRGB::Red;
  }
  FastLED.show();

  gyro_range = imu_prefs.getUChar("gyro_range");
  setGyroRange(gyro_handle, gyro_range);
  accel_range = imu_prefs.getUChar("accel_range");
  setAccelRange(accel_handle, accel_range);

  accel_odr = imu_prefs.getUChar("accel_odr");
  setAccelOdr(accel_handle, accel_odr);
  gyro_odr = imu_prefs.getUChar("gyro_odr");
  setGyroOdr(gyro_handle, gyro_odr);

  // initialize interrups for imu gyro and accel
  accel_handle.pinModeInt1(Bmi088Accel::PUSH_PULL, Bmi088Accel::ACTIVE_HIGH);
  accel_handle.mapDrdyInt1(true);

  gyro_handle.pinModeInt3(Bmi088Gyro::PUSH_PULL, Bmi088Gyro::ACTIVE_HIGH);
  gyro_handle.mapDrdyInt3(true);

  pinMode(INT_ACCEL, INPUT);
  attachInterrupt(INT_ACCEL, accel_drdy_int, RISING);
  pinMode(INT_GYRO, INPUT);
  attachInterrupt(INT_GYRO, gyro_drdy_int, RISING);

  for (;;)
  {
    buttons[0] = !digitalRead(BUTTON0_PIN);
    buttons[1] = !digitalRead(BUTTON1_PIN);
    buttons[2] = !digitalRead(BUTTON2_PIN);

    // synchronized reading requires setting up the interrupt of one sensor triggering the other
    // if required this can be implemented in the future as in the example in the BMI088 library
    bool synchronized_read = false;
    if (synchronized_read)
    {
      if (accel_drdy_flag && gyro_drdy_flag)
      {
        int64_t current_update_time = esp_timer_get_time();
        float dt = (float)(last_gyro_update - current_update_time) / 1e6;
        last_gyro_update = current_update_time;

        gyro_handle.readSensor();
        gyro_drdy_flag = false;

        accel_handle.readSensor();
        accel_drdy_flag = false;

        float tmp_gyro[3];
        tmp_gyro[0] = gyro_handle.getGyroX_rads();
        tmp_gyro[1] = gyro_handle.getGyroY_rads();
        tmp_gyro[2] = gyro_handle.getGyroZ_rads();

        float tmp_accel[3];
        tmp_accel[0] = accel_handle.getAccelX_mss();
        tmp_accel[1] = accel_handle.getAccelY_mss();
        tmp_accel[2] = accel_handle.getAccelZ_mss();

        filter_.update(tmp_accel[0], tmp_accel[1], tmp_accel[2], tmp_gyro[0], tmp_gyro[1], tmp_gyro[2], dt);

        if (isnan(tmp_gyro[0]) || isnan(tmp_gyro[1]) || isnan(tmp_gyro[2]))
          return;
        gyro[0] = tmp_gyro[0];
        gyro[1] = tmp_gyro[1];
        gyro[2] = tmp_gyro[2];

        if (isnan(tmp_accel[0]) || isnan(tmp_accel[1]) || isnan(tmp_accel[2]))
          continue;
        accel[0] = tmp_accel[0];
        accel[1] = tmp_accel[1];
        accel[2] = tmp_accel[2];

        double q0, q1, q2, q3;
        filter_.getOrientation(q0, q1, q2, q3);
        quat[0] = q1;
        quat[1] = q2;
        quat[2] = q3;
        quat[3] = q0;
      }
    }
    else
    { // not synchronized
      if (accel_drdy_flag)
      {
        accel_drdy_flag = false;
        accel_handle.readSensor();
        float tmp_accel[3];
        tmp_accel[0] = accel_handle.getAccelX_mss();
        tmp_accel[1] = accel_handle.getAccelY_mss();
        tmp_accel[2] = accel_handle.getAccelZ_mss();
        if (isnan(tmp_accel[0]) || isnan(tmp_accel[1]) || isnan(tmp_accel[2]))
          return;

        accel[0] = tmp_accel[0];
        accel[1] = tmp_accel[1];
        accel[2] = tmp_accel[2];

        filter_.update_acc(accel[0], accel[1], accel[2]);

        double q0, q1, q2, q3;
        filter_.getOrientation(q0, q1, q2, q3); // hamilton to ros quaternion
        quat[0] = q1;
        quat[1] = q2;
        quat[2] = q3;
        quat[3] = q0;
      }
      if (gyro_drdy_flag)
      {
        gyro_drdy_flag = false;

        gyro_handle.readSensor();
        float tmp_gyro[3];
        tmp_gyro[0] = gyro_handle.getGyroX_rads();
        tmp_gyro[1] = gyro_handle.getGyroY_rads();
        tmp_gyro[2] = gyro_handle.getGyroZ_rads();

        if (isnan(tmp_gyro[0]) || isnan(tmp_gyro[1]) || isnan(tmp_gyro[2]))
          return;

        gyro[0] = tmp_gyro[0];
        gyro[1] = tmp_gyro[1];
        gyro[2] = tmp_gyro[2];

        int64_t current_update_time = esp_timer_get_time();
        float dt = (float)(current_update_time - last_gyro_update) / 1e6;
        last_gyro_update = current_update_time;
        filter_.update_gyro(gyro[0], gyro[1], gyro[2], dt);
        double q0, q1, q2, q3;
        filter_.getOrientation(q0, q1, q2, q3); // hamilton to ros quaternion
        quat[0] = q1;
        quat[1] = q2;
        quat[2] = q3;
        quat[3] = q0;
      }
    }
  }
}
