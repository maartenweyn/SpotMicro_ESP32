#ifndef BT_SPP_H
#define BT_SPP_H

#include "esp_err.h"

#define DEVICE_NAME "ESP_SPOT_MICRO"

#define DATA_BUFFER_LENGTH  512

/* Attributes State Machine */

enum
{
    IDX_SVC,
    
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,

    IDX_CHAR_B,
    IDX_CHAR_VAL_B,
    IDX_CHAR_CFG_B,

    // IDX_CHAR_C,
    // IDX_CHAR_VAL_C,
    // IDX_CHAR_CFG_C,

    IDX_NB,
};

typedef void (*new_orientation_cb_t)(int16_t omega, int16_t phi, int16_t psi, int16_t xm, int16_t ym, int16_t zm);

void start_bluetooth();
void set_new_orientation_cb(new_orientation_cb_t cb);
esp_err_t set_new_orientation_act_value(int16_t omega, int16_t phi, int16_t psi, int16_t xm, int16_t ym, int16_t zm);

#endif