#include "servo.h"
#include "config.h"

#include "pca9685.h"

#include <esp_log.h>

static char tag[] = "SERVO";


void init_servos() {

}

esp_err_t set_servo(uint8_t id, uint16_t angle) {
  esp_err_t ret;
  uint16_t pulse = (uint16_t) (0.5 + servo_min[id] + (angle * servo_conversion[id]));
  ESP_LOGD(tag, "setPWM of servo %d, %d degrees -> Pulse %d", id, angle, pulse);
  ret = setPWM(id, 0, pulse);

  if (ret == ESP_OK) return ESP_OK;
  else return ESP_FAIL;
}

