#include <string.h>
#include "esp_log.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "eeg_driver.h"

#define RETURN_ON_ERROR(x) if (x != ESP_OK) return x

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

#define UNUSED -1
#define SPI_MOSI_PIN 12
#define SPI_SCLK_PIN 15
#define SPI_MISO_PIN 13
#define CS_ENABLE_PIN 6

#define SPI_CLK_FREQ 4E6//DO NOT EXCEED 4E6 - See 9.5.3.1 of ADS1299 datasheet
#define SPI_QUEUE_SIZE 16
#define SPI_MODE 1
#define SPI_HOST_LOCAL SPI2_HOST

//Command definitions (Table 10)
#define RESET 0x06
#define WAKEUP 0x02
#define STANDBY 0x04
#define START 0x08
#define STOP 0x0A
#define RREG1_UPPER 0x20
#define RREG1_LOWER_MASK 0x1F

//Register map (Table 11)
#define ID 0x00
#define CONFIG1 0x01
#define CONFIG2 0x02
#define CONFIG3 0x03
#define LOFF 0x04
#define CH1SET 0x05
#define CH2SET 0x06
#define CH3SET 0x07
#define CH4SET 0x08
#define CH5SET 0x09
#define CH6SET 0x0A
#define CH7SET 0x0B
#define CH8SET 0x0C
#define BIAS_SENSP 0x0D
#define BIAS_SENSN 0x0E
#define LOFF_SENSP 0x0F
#define LOFF_SENSN 0x10
#define LOFF_FLIP 0x11
#define LOFF_STATP 0x12
#define LOFF_STATN 0x13
#define GPIO 0x14
#define MISC1 0x15
#define MISC2 0x16
#define CONFIG4 0x17

typedef struct {
  spi_device_handle_t handle;
  uint8_t id;
} eeg_t;

static const char tag[] = "eeg_driver";
eeg_t status;

static esp_err_t write_command(spi_device_handle_t handle, uint8_t data) {
  spi_transaction_t transaction = {
    .flags = SPI_TRANS_USE_TXDATA,
    .length = sizeof(data) * 8,
  };

  memcpy(transaction.tx_data, &data, sizeof(data));

  ESP_LOGD(tag, "Wrote: "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(data));

  return spi_device_transmit(handle, &transaction);
}

static esp_err_t read_register(spi_device_handle_t handle, uint8_t address, uint8_t* dataOut) {
  uint8_t command[2] = {RREG1_UPPER | (address & RREG1_LOWER_MASK), 1};

  spi_transaction_t transaction = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .length = sizeof(command)*8,
    .rxlength = sizeof(*dataOut)*8,
  };

  memcpy(transaction.tx_data, command, sizeof(command));

  esp_err_t err = spi_device_transmit(handle, &transaction);
  memcpy(dataOut, transaction.rx_data, sizeof(*dataOut));

  return err;
}

esp_err_t reset() {
  return write_command(status.handle, RESET);
}

esp_err_t wakeup() {
  return write_command(status.handle, WAKEUP);
}

esp_err_t standby() {
  return write_command(status.handle, STANDBY);
}

esp_err_t start() {
  return write_command(status.handle, START);
}

esp_err_t stop() {
  return write_command(status.handle, STOP);
}

esp_err_t eeg_dev_init(void) {
  const spi_bus_config_t spi_config = {
    .mosi_io_num = SPI_MOSI_PIN,
    .miso_io_num = SPI_MISO_PIN,
    .sclk_io_num = SPI_SCLK_PIN,
    .quadwp_io_num = UNUSED,
    .quadhd_io_num = UNUSED,
    .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_DUAL,
  };
  RETURN_ON_ERROR(spi_bus_initialize(SPI_HOST_LOCAL, &spi_config, SPI_DMA_AUTO));

  const spi_device_interface_config_t dev_config = {
    .clock_speed_hz = SPI_CLK_FREQ,
    .queue_size = SPI_QUEUE_SIZE,
    .mode = SPI_MODE,
    .spics_io_num = CS_ENABLE_PIN,
  };
  RETURN_ON_ERROR(spi_bus_add_device(SPI_HOST_LOCAL, &dev_config, &status.handle));

  //Initial device commands
  RETURN_ON_ERROR(reset());

  vTaskDelay(1 / portTICK_PERIOD_MS);

  //RETURN_ON_ERROR(self_test(&status));

  ESP_LOGI(tag, "Initialized");

  return ESP_OK;
}

esp_err_t print_id() {
  RETURN_ON_ERROR(read_register(status.handle, 0x00, &status.id));
  ESP_LOGI(tag, "ID: "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(status.id));
  return ESP_OK;
}