#include "uart_port_handler.h"

static intr_handle_t uart_isr_intr_handle;

void IRAM_ATTR static_uart_isr(void *arg)
{
  ESP32UartPortHandler *uart_port = (ESP32UartPortHandler *)arg;

  uart_dev_t *uart_dev = ESP32_UART_DEVICES[uart_port->uart_num_];

  uint16_t status = uart_dev->int_st.val;
  uint16_t rx_fifo_len = uart_dev->status.rxfifo_cnt;

  for (int i = 0; i < rx_fifo_len; i++)
  { // evtl eine while schleife mit der unteren bedingung
    uart_port->buffer_[uart_port->write_pointer_] = uart_dev->fifo.rw_byte;
    uart_port->write_pointer_ = (uart_port->write_pointer_ + 1) % UART_BUFFER_SIZE;
  }
  while (uart_dev->status.rxfifo_cnt || (uart_dev->mem_rx_status.wr_addr != uart_dev->mem_rx_status.rd_addr))
  {
    uart_dev->fifo.rw_byte;
  }
  uart_clear_intr_status(uart_port->uart_num_, status);
}

ESP32UartPortHandler::ESP32UartPortHandler(uint8_t uart_num, uint8_t tx_pin, uint8_t rx_pin, uint8_t dir_pin)
{
  uart_num_ = uart_num;
  tx_pin_ = tx_pin;
  rx_pin_ = rx_pin;
  dxl_dir_pin_ = dir_pin; // TODO HW Flow control
}

void ESP32UartPortHandler::setBaudRate(int baud_rate)
{
  baud_rate_ = baud_rate;
}

void ESP32UartPortHandler::setItrParams(uint8_t rx_timeout_thresh, uint8_t txfifo_empty_intr_thresh, uint8_t rxfifi_full_thresh) {
  rx_timeout_thresh_ = rx_timeout_thresh;
  txfifo_empty_intr_thresh_ = txfifo_empty_intr_thresh;
  rxfifo_full_thresh_ = rxfifi_full_thresh;
}

void ESP32UartPortHandler::begin()
{

  const uart_config_t uart_config = {
      .baud_rate = baud_rate_,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };
  const uart_intr_config_t uart_intr_config_struct = {
      .intr_enable_mask = UART_RXFIFO_FULL_INT_ENA | UART_RXFIFO_TOUT_INT_ENA_M,
      .rx_timeout_thresh = rx_timeout_thresh_,
      .txfifo_empty_intr_thresh = txfifo_empty_intr_thresh_,
      .rxfifo_full_thresh = rxfifo_full_thresh_,
  };

  uart_driver_install(uart_num_, 256 * 2, 0, 0, NULL, 0);
  uart_param_config(uart_num_, &uart_config);
  uart_intr_config(uart_num_, &uart_intr_config_struct);
  uart_set_pin(uart_num_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  // uart_set_mode(uart_num_, UART_MODE_RS485_HALF_DUPLEX);
  uart_disable_tx_intr(uart_num_);
  uart_disable_rx_intr(uart_num_);
  uart_isr_free(uart_num_);
  uart_isr_register(uart_num_, &static_uart_isr, this, ESP_INTR_FLAG_IRAM, &uart_isr_intr_handle);
  uart_enable_rx_intr(uart_num_);
  setOpenState(true);
}

void ESP32UartPortHandler::end()
{ // not tested, probably never going to be called
  uart_disable_rx_intr(uart_num_);
  uart_isr_free(uart_num_);
  uart_driver_delete(uart_num_);
}

int ESP32UartPortHandler::available()
{
  if (write_pointer_ >= read_pointer_)
  {
    return write_pointer_ - read_pointer_;
  }
  else
  {
    return UART_BUFFER_SIZE - read_pointer_ + write_pointer_;
  }
}

int ESP32UartPortHandler::read()
{
  uint8_t data = buffer_[read_pointer_];
  read_pointer_ = (read_pointer_ + 1) % UART_BUFFER_SIZE;
  return data;
}

size_t ESP32UartPortHandler::write(uint8_t byte)
{
  return uart_write_bytes(uart_num_, (const char *)&byte, 1);
}
size_t ESP32UartPortHandler::write(uint8_t *packet, size_t length)
{
  digitalWrite(dxl_dir_pin_, HIGH);
  int num_bytes = uart_write_bytes(uart_num_, packet, length);
  while (ESP32_UART_DEVICES[uart_num_]->status.st_utx_out != 0)
  {
    // wait for the fifo to be empty
  }
  //
  /*while(ESP32_UART_DEVICES[uart_num_]->status.txfifo_cnt > 0) {
    // wait for the fifo to be empty
  }*/
  digitalWrite(dxl_dir_pin_, LOW);
  return num_bytes;
}

int ESP32UartPortHandler::getNumItr()
{
  return intr_cnt;
}
