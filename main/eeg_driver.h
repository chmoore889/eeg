#ifndef H_FREQ_CTRL_
#define H_FREQ_CTRL_

#include "esp_err.h"

esp_err_t eeg_dev_init(void);
esp_err_t print_id(void);
esp_err_t reset(void);
esp_err_t wakeup(void);
esp_err_t standby(void);
esp_err_t start(void);
esp_err_t stop(void);

#endif