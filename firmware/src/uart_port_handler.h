#include <Arduino.h>
#include <utility/port_handler.h>
#include <driver/uart.h>
#include <soc/uart_reg.h>
#include <soc/uart_struct.h>

#ifndef ESP32UARTPORTHANDLER_H
#define ESP32UARTPORTHANDLER_H

#define UART_BUFFER_SIZE 1024

static uart_dev_t* ESP32_UART_DEVICES[] = {&UART0, &UART1, &UART2};


class ESP32UartPortHandler : public DXLPortHandler{
  public:
    ESP32UartPortHandler(uint8_t uart_num, uint8_t tx_pin, uint8_t rx_pin, uint8_t dir_pin);
    void begin();
    void end();
    int available();
    int read();
    size_t write(uint8_t byte);
    size_t write(uint8_t* packet, size_t length);
    int getNumItr();
    void setBaudRate(int baud_rate);

    uint8_t buffer_[UART_BUFFER_SIZE] = {0};
    uint8_t uart_num_;
    uint32_t write_pointer_ = 0;
    uint32_t read_pointer_ = 0;
    int intr_cnt = 0;

  private:
    void IRAM_ATTR uart_isr(void* arg);


    uint8_t tx_pin_;
    uint8_t rx_pin_;
    uint8_t dxl_dir_pin_;

    int baud_rate_;

};

#endif // ESP32UARTPORTHANDLER_H