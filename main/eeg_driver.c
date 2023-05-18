#include <string.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "eeg_driver.h"
#include "eeg_driver_int.h"

#define EXTRACT_BITS(data, lowBit, size) ((data >> lowBit) & ((1 << size) - 1))

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)                                                   \
  (byte & 0x80 ? '1' : '0'), (byte & 0x40 ? '1' : '0'),                        \
      (byte & 0x20 ? '1' : '0'), (byte & 0x10 ? '1' : '0'),                    \
      (byte & 0x08 ? '1' : '0'), (byte & 0x04 ? '1' : '0'),                    \
      (byte & 0x02 ? '1' : '0'), (byte & 0x01 ? '1' : '0')

#define UNUSED -1
#define SPI_MOSI_PIN 12
#define SPI_SCLK_PIN 15
#define SPI_MISO_PIN 13
#define CS_ENABLE_PIN 6

#define RESET_PIN 5
#define DRDY_PIN 4

#define SPI_CLK_FREQ                                                           \
  1000 // DO NOT EXCEED 4E6 - See 9.5.3.1 of ADS1299
       // datasheet
#define SPI_QUEUE_SIZE 16
#define SPI_MODE 1
#define SPI_HOST_LOCAL SPI2_HOST

static esp_err_t drdy_triggered(void);

typedef struct {
  spi_device_handle_t handle;
  uint8_t num_channels;
} eeg_t;

static const char tag[] = "eeg_driver";
static eeg_t status = {0};

uint8_t getNumChannels(void) { return status.num_channels; }

static esp_err_t write_command(spi_device_handle_t handle, uint8_t data) {
  spi_transaction_t transaction = {
      .flags = SPI_TRANS_USE_TXDATA,
      .length = sizeof(data) * 8,
  };

  memcpy(transaction.tx_data, &data, sizeof(data));

  ESP_LOGD(tag, "Wrote: " BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(data));

  return spi_device_transmit(handle, &transaction);
}

static esp_err_t read_register(spi_device_handle_t handle, uint8_t address,
                               uint8_t *dataOut) {
  const uint8_t command[2] = {RREG_UPPER | (address & RREG_LOWER_MASK), 1};

  spi_transaction_t transaction = {
      .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
      .length = sizeof(command) * 8,
      .rxlength = sizeof(*dataOut) * 8,
  };

  memcpy(transaction.tx_data, command, sizeof(command));

  esp_err_t err = spi_device_transmit(handle, &transaction);
  memcpy(dataOut, transaction.rx_data, sizeof(*dataOut));
  ESP_LOGD(tag, "Read %u", *dataOut);

  return err;
}

static esp_err_t write_register(spi_device_handle_t handle, uint8_t address,
                                uint8_t data) {
  const uint8_t command[2] = {WREG_UPPER | (address & WREG_LOWER_MASK), 1};
  const size_t length_bytes = sizeof(command) + sizeof(data);

  spi_transaction_t transaction = {
      .flags = SPI_TRANS_USE_TXDATA,
      .length = length_bytes * 8,
  };

  memcpy(transaction.tx_data, command, sizeof(command));
  memcpy(transaction.tx_data + sizeof(command), &data, sizeof(data));

  return spi_device_transmit(handle, &transaction);
}

esp_err_t reset() { return write_command(status.handle, RESET); }

esp_err_t wakeup() { return write_command(status.handle, WAKEUP); }

esp_err_t standby() { return write_command(status.handle, STANDBY); }

esp_err_t start() { return write_command(status.handle, START); }

esp_err_t stop() { return write_command(status.handle, STOP); }

esp_err_t read_data_continuous(void) {
  return write_command(status.handle, RDATAC);
}

esp_err_t stop_data_continuous(void) {
  return write_command(status.handle, SDATAC);
}

esp_err_t read_data(void) { return write_command(status.handle, RDATA); }

// Resets EEG device and sets configuration registers
// Initial settings are as follows:
// Enable internal reference, set internal bias reference, enable bias buffer
// Enable SRB1 (referential montage)
esp_err_t eeg_dev_init(void) {
  const spi_bus_config_t spi_config = {
      .mosi_io_num = SPI_MOSI_PIN,
      .miso_io_num = SPI_MISO_PIN,
      .sclk_io_num = SPI_SCLK_PIN,
      .quadwp_io_num = UNUSED,
      .quadhd_io_num = UNUSED,
      .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK |
               SPICOMMON_BUSFLAG_DUAL,
  };
  ESP_RETURN_ON_ERROR(
      spi_bus_initialize(SPI_HOST_LOCAL, &spi_config, SPI_DMA_CH_AUTO), tag,
      "Failed to initialize SPI bus");

  const spi_device_interface_config_t dev_config = {
      .clock_speed_hz = SPI_CLK_FREQ,
      .queue_size = SPI_QUEUE_SIZE,
      .mode = SPI_MODE,
      .spics_io_num = CS_ENABLE_PIN,
      .flags = SPI_DEVICE_HALFDUPLEX
  };
  ESP_RETURN_ON_ERROR(
      spi_bus_add_device(SPI_HOST_LOCAL, &dev_config, &status.handle), tag,
      "Failed to add SPI device");

  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT_OD;
  io_conf.pin_bit_mask = (1ULL<<RESET_PIN);
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  ESP_RETURN_ON_ERROR(gpio_config(&io_conf), tag, "Failed to config GPIO");
  gpio_set_level(RESET_PIN, 1);

  //Wait for supplies and VCAP1 to stabilize
  vTaskDelay(1 / portTICK_PERIOD_MS);

  // Initial device commands
  // ESP_RETURN_ON_ERROR(reset(), tag, "Failed to reset device");
  // ESP_LOGD(tag, "Sent reset command");

  gpio_set_level(RESET_PIN, 0);
  vTaskDelay(portTICK_PERIOD_MS);
  gpio_set_level(RESET_PIN, 1);
  vTaskDelay(portTICK_PERIOD_MS);
  

  vTaskDelay(portTICK_PERIOD_MS);

  // Send SDATAC command to stop continuous data and allow for other commands
  // See figure 67 in ADS1299 datasheet
  ESP_RETURN_ON_ERROR(stop_data_continuous(), tag,
                      "Failed to stop initial data");
  ESP_LOGD(tag, "Stopped initial data");
TempTest:
vTaskDelay(portTICK_PERIOD_MS);
  // Read ID register
  uint8_t rawIDReg;
  ESP_RETURN_ON_ERROR(read_register(status.handle, ID, &rawIDReg), tag,
                      "Failed to read ID");

  // Extract and check device ID from ID register
  const uint8_t extractedId = EXTRACT_BITS(rawIDReg, 2, 2);
  if (extractedId != ADS1299_DEVICE_ID) {
    ESP_LOGE(tag, "Device ID is incorrect. Expected %u, got %u",
             ADS1299_DEVICE_ID, extractedId);
    goto TempTest;
    return ESP_FAIL;
  }
  ESP_LOGD(tag, "Device ID is correct");

  // Extract number of channels from ID register
  const uint8_t numChannelCode = EXTRACT_BITS(rawIDReg, 0, 2) + 1;

  // See 9.6.1.1 in ADS1299 datasheet for codes
  switch (numChannelCode) {
  case 0b00:
    status.num_channels = 4;
    break;
  case 0b01:
    status.num_channels = 6;
    break;
  case 0b10:
    status.num_channels = 8;
    break;
  }
  ESP_LOGD(tag, "Number of channels: %u", status.num_channels);

  // Enable internal reference, set internal bias reference, enable bias buffer
  ESP_RETURN_ON_ERROR(write_register(status.handle, CONFIG3,
                                     ADS1299_REG_CONFIG3_REFBUF_ENABLED |
                                         ADS1299_REG_CONFIG3_BIASREF_INT |
                                         ADS1299_REG_CONFIG3_BIASBUF_ENABLED),
                      tag, "Failed to write CONFIG3");

  // Enable SRB1 (referential montage)
  ESP_RETURN_ON_ERROR(
      write_register(status.handle, MISC1, ADS1299_REG_MISC1_SRB1_ON), tag,
      "Failed to write MISC1");

  ESP_LOGI(tag, "Initialized EEG device");
  return ESP_OK;
}

esp_err_t drdy_triggered() { return ESP_OK; }