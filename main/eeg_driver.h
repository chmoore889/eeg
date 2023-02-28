#ifndef H_EEG_DRIVER_
#define H_EEG_DRIVER_

#include "esp_err.h"

esp_err_t eeg_dev_init(void);
uint8_t getNumChannels(void);

//Direct commands
esp_err_t reset(void);
esp_err_t wakeup(void);
esp_err_t standby(void);
esp_err_t start(void);
esp_err_t stop(void);
esp_err_t read_data_continuous(void);
esp_err_t stop_data_continuous(void);
esp_err_t read_data(void);

#endif