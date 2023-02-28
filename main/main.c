#include "esp_log.h"
#include "eeg_driver.h"

static const char tag[] = "main";

void app_main(void) {
  esp_err_t err = eeg_dev_init();
  if (err != ESP_OK) {
    ESP_LOGE(tag, "Failed to initialize EEG device");
    return;
  }

  print_id();
  if(err != ESP_OK) {
    ESP_LOGE(tag, "Failed to get ID");
    return;
  }
}
