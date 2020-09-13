#ifndef SERVO_H
#define SERVO_H


#include <esp_err.h>

typedef struct {
  uint16_t pulse_0; 
  uint16_t pulse_180; 
  int8_t invert;
} servo_settings_t;


void init_servos();
esp_err_t set_servo(uint8_t id, uint16_t angle);

#endif