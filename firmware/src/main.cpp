#include <main.h>


// define two tasks for reading the dxl bus and doing other work
void TaskDXL( void *pvParameters );
void TaskWorker( void *pvParameters );

TaskHandle_t th_dxl,th_worker;

Preferences dxl_prefs;
Preferences imu_prefs;

/*---------------------- DXL defines and variables ---------------------*/


uart_t* uart;
DYNAMIXEL::FastSlave dxl(DXL_MODEL_NUM, DXL_DIR_PIN, DXL_PROTOCOL_VER_2_0);




// id and baud are stored using preferences library for persistance between resets of the chip
uint8_t id;
uint8_t baud;


/*---------------------- IMU defines and variables ---------------------*/

Bmi088Accel accel_handle(SPI, ACCEL_CS);
Bmi088Gyro gyro_handle(SPI, GYRO_CS);

// flag triggered by interrupt to read data from IMU
volatile bool accel_drdy_flag, gyro_drdy_flag = false;

// lists for storing measurements and filter outputs
float gyro[3] = {0,0,0};
float accel[3] = {0,0,0};
float quat[4] = {0,0,0,0};

// registers for parameters of IMU
uint8_t gyro_range, accel_range;

// flags which can be set from the dxl bus to trigger calibration of gyro bias
bool calibrate_gyro = false;
bool reset_gyro_calibration = false;

// registers for parameters of complementary filter
bool do_adaptive_gain, do_bias_estimation;
float accel_gain, bias_alpha;

imu_tools::ComplementaryFilter filter_;
unsigned long last_gyro_update = esp_timer_get_time();

/*---------------------- LEDS ---------------------*/

CRGB leds[NUM_LEDS];

/*---------------------- Buttons ---------------------*/

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

  //#ifdef DEBUG
    DEBUG_SERIAL.begin(115200);
  //#endif
  
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
    case 7: real_baud = 4500000; break;
  }
  DEBUG_SERIAL.print("baud: ");
  DEBUG_SERIAL.println(real_baud);
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
      // TODO
      delay(1); //make sure processing of imu is done for 1 cycle
      //int status = IMU.calibrateGyro();
      /*if(status == 1)
      {
        leds[0] = CRGB::Green;
      }
      else
      {
        leds[0] = CRGB::Red;
      }*/
      delay(200);
      leds[1] = CRGB::Black;
      FastLED.show();
      calibrate_gyro = false;
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
    imu_prefs.putFloat("gain_accel", accel_gain);
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
  if(dxl_prefs.getUChar("init") != 43) // check if prefs are initialized
  {
    Serial.println("Initializing dxl prefs");
    dxl_prefs.putUChar("id", DEFAULT_ID);
    dxl_prefs.putUChar("baud", DEFAULT_BAUD);
    dxl_prefs.putUChar("init", 42); // set initialized
  } else {
    Serial.println("dxl prefs already initialized");
  }
  id = dxl_prefs.getUChar("id");
  baud = dxl_prefs.getUChar("baud");

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

  dxl.addControlItem(ADDR_CONTROL_ITEM_GYRO_X, gyro[0]); //TODO: 16 bit anstatt float
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
  
  dxl.addControlItem(ADDR_CONTROL_ITEM_GYRO_RANGE, gyro_range);
  dxl.addControlItem(ADDR_CONTROL_ITEM_ACCEL_RANGE, accel_range);
  
  dxl.addControlItem(ADDR_CONTROL_ITEM_CALIBRATE_GYRO, calibrate_gyro);
  dxl.addControlItem(ADDR_CONTROL_ITEM_RESET_GYRO_CALIBRATION, reset_gyro_calibration);

  // TODO configure ODR
  dxl.addControlItem(ADDR_CONTROL_ITEM_DO_ADAPTIVE_GAIN, do_adaptive_gain);
  dxl.addControlItem(ADDR_CONTROL_ITEM_DO_BIAS_ESTIMATION, do_bias_estimation);
  dxl.addControlItem(ADDR_CONTROL_ITEM_ACCEL_GAIN, accel_gain);
  dxl.addControlItem(ADDR_CONTROL_ITEM_BIAS_ALPHA, bias_alpha);

  dxl.setWriteCallbackFunc(write_callback_func);
  
  pinMode(DXL_DIR_PIN, OUTPUT);
  digitalWrite(DXL_DIR_PIN, LOW);
  // init uart 0, given baud, 8bits 1stop no parity, rx pin 14, tx pin 27, 256 buffer, no inversion
  // uart_t* uartBegin(uint8_t uart_nr, uint32_t baudrate, uint32_t config, int8_t rxPin, int8_t txPin, uint16_t rx_buffer_size, uint16_t tx_buffer_size, bool inverted, uint8_t rxfifo_full_thrhd)
  uart = uartBegin(1, dxl_to_real_baud(baud), SERIAL_8N1, DXL_U2_RX_PIN, DXL_U2_TX_PIN,  256, 256, false, 122);
  // disable all interrupts
  uart->dev->conf1.rx_tout_en = 0;
  uart->dev->int_ena.rxfifo_full = 0;
  uart->dev->int_ena.frm_err = 0;
  uart->dev->int_ena.rxfifo_tout = 0;
  /*
  for (;;)
  {
    if (uart->dev->status.rxfifo_cnt != 0 || (uart->dev->mem_rx_status.wr_addr != uart->dev->mem_rx_status.rd_addr)) {
      char c = uart->dev->fifo.rw_byte;
      DEBUG_SERIAL.print(c, HEX);
    }
  }
    */
  for (;;)
  {
    
    if(dxl.processPacket(uart)){
      if(dxl.getID() != id) // since we cant add the id as a control item, we need to check if it has been updated manually
      {
        id = dxl.getID();
        dxl_prefs.putUChar("id", id);
      }
    }
    //#ifdef DEBUG
    else {
        DEBUG_SERIAL.print("Last lib err code: ");
        DEBUG_SERIAL.print(dxl.getLastLibErrCode());
        DEBUG_SERIAL.print(", ");
        DEBUG_SERIAL.print("Last status packet err code: ");
        DEBUG_SERIAL.println(dxl.getLastStatusPacketError());
        DEBUG_SERIAL.println();
    }
    //#endif
  }

}

/*---------------------- IMU ---------------------*/
void setAccelRange(uint8_t range)
{
    Bmi088Accel::Range accel_range;
    switch(range)
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
        accel_range = ACCEL_RANGE_DEFAULT;
    }
    accel_handle.setRange(accel_range);
}
void setGyroRange(uint8_t range)
{
    Bmi088Gyro::Range gyro_range;
    switch(range)
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
        gyro_range = GYRO_RANGE_DEFAULT;
    }
    gyro_handle.setRange(gyro_range);
}

void IRAM_ATTR accel_drdy_int() //IRAM_ATTR puts this function into ram, required since it is called as an interrupt
{
  accel_drdy_flag = true;
}

void IRAM_ATTR gyro_drdy_int() //IRAM_ATTR puts this function into ram, required since it is called as an interrupt
{
  gyro_drdy_flag = true;
}

void TaskWorker(void *pvParameters) 
{
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  (void) pvParameters;
  imu_prefs.begin("imu");
  if(imu_prefs.getUChar("init_imu") != 43) // check if prefs are initialized
  {
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

  // setup buttons
  pinMode(BUTTON0_PIN,INPUT_PULLUP);
  pinMode(BUTTON1_PIN,INPUT_PULLUP);
  
  int accel_status = accel_handle.begin();
  int gyro_status = gyro_handle.begin();

  #ifdef DEBUG
  
  DEBUG_SERIAL.print("Accel status: ");
  DEBUG_SERIAL.println(accel_status);
  DEBUG_SERIAL.print("Gyro status: ");
  DEBUG_SERIAL.println(gyro_status);
  #endif

  if(accel_status == 1 && gyro_status == 1) {
    leds[2] = CRGB::Green;
  }
  else if (accel_status != 1) {
    leds[1] = CRGB::Red;
  } else if (gyro_status != 1) {
    leds[2] = CRGB::Red;
  }
  FastLED.show();

  gyro_range = imu_prefs.getUChar("gyro_range");
  //setGyroRange(gyro_range);
  gyro_handle.setRange(GYRO_RANGE_DEFAULT);
  accel_range = imu_prefs.getUChar("accel_range");
  //setAccelRange(accel_range);
  accel_handle.setRange(ACCEL_RANGE_DEFAULT);

  accel_handle.setOdr(ACCEL_ODR_DEFAULT);
  gyro_handle.setOdr(GYRO_ODR_DEFAULT);

  // initialize interrups for imu gyro and accel
  accel_handle.pinModeInt1(Bmi088Accel::PUSH_PULL,Bmi088Accel::ACTIVE_HIGH);
  accel_handle.mapDrdyInt1(true);
  
  gyro_handle.pinModeInt3(Bmi088Gyro::PUSH_PULL,Bmi088Gyro::ACTIVE_HIGH);
  gyro_handle.mapDrdyInt3(true);

  pinMode(INT_ACCEL,INPUT);
  attachInterrupt(INT_ACCEL,accel_drdy_int,RISING);
  pinMode(INT_GYRO,INPUT);
  attachInterrupt(INT_GYRO,gyro_drdy_int,RISING);  

  for (;;)
  {
    buttons[0] = !digitalRead(BUTTON0_PIN);
    buttons[1] = !digitalRead(BUTTON1_PIN);
    if (accel_drdy_flag){
      accel_drdy_flag = false;
      accel_handle.readSensor();
      float tmp_accel[3];
      tmp_accel[0] = accel_handle.getAccelX_mss();
      tmp_accel[1] = accel_handle.getAccelY_mss();
      tmp_accel[2] = accel_handle.getAccelZ_mss();
      if(isnan(tmp_accel[0]) || isnan(tmp_accel[1]) || isnan(tmp_accel[2]))
        return;
      // copy data to global variables, no loops because compiler might not optimize them out
      accel[0] = tmp_accel[0];
      accel[1] = tmp_accel[1];
      accel[2] = tmp_accel[2];

      filter_.update_acc(accel[0], accel[1], accel[2]);

      double q0,q1,q2,q3;
      filter_.getOrientation(q0, q1, q2, q3); //hamilton to ros quaternion
      quat[0] = q1;
      quat[1] = q2;
      quat[2] = q3;
      quat[3] = q0;
      // #if DEBUG
      //   DEBUG_SERIAL.println("accel");
      // #endif //DEBUG
    }

    if (gyro_drdy_flag){
      gyro_drdy_flag = false;
  
      gyro_handle.readSensor();
      float tmp_gyro[3];
      tmp_gyro[0] = gyro_handle.getGyroX_rads();
      tmp_gyro[1] = gyro_handle.getGyroY_rads();
      tmp_gyro[2] = gyro_handle.getGyroZ_rads();

      if(isnan(tmp_gyro[0]) || isnan(tmp_gyro[1]) || isnan(tmp_gyro[2]))
        return;      
      // copy data to global variables, no loops because compiler might not optimize them out
      gyro[0] = tmp_gyro[0];
      gyro[1] = tmp_gyro[1];
      gyro[2] = tmp_gyro[2];
      unsigned long current_update_time = esp_timer_get_time();
      float dt = (float) (last_gyro_update - current_update_time) / 1e6;
      last_gyro_update = current_update_time;
      filter_.update_gyro(gyro[0], gyro[1], gyro[2], dt);
      double q0,q1,q2,q3;
      filter_.getOrientation(q0, q1, q2, q3); //hamilton to ros quaternion
      quat[0] = q1;
      quat[1] = q2;
      quat[2] = q3;
      quat[3] = q0;
      // #if DEBUG
      //   DEBUG_SERIAL.println("gyro");
      // #endif //DEBUG
    }
    if (true) {
      #if DEBUG
        DEBUG_SERIAL.print(gyro[0]);
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.print(gyro[1]);
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.print(gyro[2]);
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.print(accel[0]);
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.print(accel[1]);
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.print(accel[2]);
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.print(quat[0]);
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.print(quat[1]);
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.print(quat[2]);
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.print(quat[3]);
        DEBUG_SERIAL.print("\n");
      #endif
    }
  }
}
