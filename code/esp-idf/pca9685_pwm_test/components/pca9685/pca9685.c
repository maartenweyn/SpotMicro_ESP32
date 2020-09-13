/*************************************************** 
  This is a library for the PCA9685 LED PWM Driver

  This chip is connected via I2C, 2 pins are required to interface. The PWM frequency is set for all pins, the PWM for each individually. The set PWM is active as long as the chip is powered.

  Adapted from Jonas Scharpf <jonas@brainelectronics.de>
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "pca9685.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <driver/i2c.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "esp_err.h"
#include <errno.h>
#include "esp_log.h"
#include "esp_system.h"

uint8_t PCA9685_ADDR = 0x0;
static char tag[] = "PCA9685";

/**
 * @brief      Sets the adress where the PCA9685 is located
 *
 * @param[in]  addr  The address
 */
void set_pca9685_adress(uint8_t addr)
{
  PCA9685_ADDR = addr;
}

/**
 * @brief      Reset the PCA9685
 * 
 * @return     result of command
 */
esp_err_t resetPCA9685(void)
{
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MODE1, ACK_CHECK_EN);   // 0x0 = "Mode register 1"
    i2c_master_write_byte(cmd, 0x80, ACK_CHECK_EN);    // 0x80 = "Reset"
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    vTaskDelay(50 / portTICK_RATE_MS);

    return ret;
}

/**
 * @brief      Write two 16 bit values to the same register on an i2c device
 *
 * @param[in]  regaddr   The register address
 * @param[in]  valueOn   The value on
 * @param[in]  valueOff  The value off
 * 
 * @return     result of command
 */
esp_err_t generic_write_i2c_register_two_words(uint8_t regaddr, uint16_t valueOn, uint16_t valueOff)
{
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, valueOn & 0xff, ACK_VAL);
    i2c_master_write_byte(cmd, valueOn >> 8, NACK_VAL);
    i2c_master_write_byte(cmd, valueOff & 0xff, ACK_VAL);
    i2c_master_write_byte(cmd, valueOff >> 8, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief      Write a 16 bit value to a register on an i2c device
 *
 * @param[in]  regaddr  The register address
 * @param[in]  value    The value
 * 
 * @return     result of command
 */
esp_err_t generic_write_i2c_register_word(uint8_t regaddr, uint16_t value)
{
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value & 0xff, ACK_VAL);
    i2c_master_write_byte(cmd, value >> 8, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief      Write a 8 bit value to a register on an i2c device
 *
 * @param[in]  regaddr  The register address
 * @param[in]  value    The value
 * 
 * @return     result of command
 */
esp_err_t generic_write_i2c_register(uint8_t regaddr, uint8_t value)
{
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief      Read two 8 bit values from the same register on an i2c device
 *
 * @param[in]  regaddr  The register address
 * @param      valueA   The first value
 * @param      valueB   The second value
 *
 * @return     result of command
 */
esp_err_t generic_read_two_i2c_register(uint8_t regaddr, uint8_t* valueA, uint8_t* valueB)
{
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, PCA9685_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, valueA, ACK_VAL);
    i2c_master_read_byte(cmd, valueB, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

/**
 * @brief      Read a 16 bit value from a register on an i2c decivde
 *
 * @param[in]  regaddr  The register address
 * @param      value    The value
 *
 * @return     result of command
 */
esp_err_t generic_read_i2c_register_word(uint8_t regaddr, uint16_t* value)
{
    esp_err_t ret;

    uint8_t valueA;
    uint8_t valueB;

    ret = generic_read_two_i2c_register(regaddr, &valueA, &valueB);
    if (ret != ESP_OK) {
        return ret;
    }

    *value = (valueB << 8) | valueA;

    return ret;
}


/**
 * @brief      Sets the frequency of PCA9685
 *
 * @param[in]  freq  The frequency
 * 
 * @return     result of command
 */
esp_err_t setFrequencyPCA9685(uint16_t freq)
{
    esp_err_t ret;

    ESP_LOGD(tag, "Setting frequency to %d",freq);

    // Send to sleep
    ret = generic_write_i2c_register(MODE1, 0x10);
    if (ret != ESP_OK) {
        return ret;
    }

    // Set prescaler
    // calculation on page 25 of datasheet
    uint8_t prescale_val = (uint8_t) (((CLOCK_FREQ / (4096.0 * freq))-1)+0.5);
    ESP_LOGD(tag, "prescale value %d", prescale_val);
    ret = generic_write_i2c_register(PRE_SCALE, prescale_val);
    if (ret != ESP_OK) {
        return ret;
    }

    // reset again
    resetPCA9685();

    // Send to sleep again
    ret = generic_write_i2c_register(MODE1, 0x10);
    if (ret != ESP_OK) {
        return ret;
    }

    // wait
    vTaskDelay(5/portTICK_PERIOD_MS);

    // Write 0xa0 for auto increment LED0_x after received cmd
    ret = generic_write_i2c_register(MODE1, 0xa0);
    if (ret != ESP_OK) {
        return ret;
    }

    return ret;
}

/**
 * @brief      Sets the pwm of the pin
 *
 * @param[in]  num   The pin number
 * @param[in]  on    On time
 * @param[in]  off   Off time
 * 
 * @return     result of command
 */
esp_err_t setPWM(uint8_t num, uint16_t on, uint16_t off)
{
    esp_err_t ret;

    uint8_t pinAddress = LED0_ON_L + LED_MULTIPLYER * num;
    ret = generic_write_i2c_register_two_words(pinAddress & 0xff, on, off);

    return ret;
}

/**
 * @brief      Gets the pwm of a pin detail
 * 
 * Have read each LED0_ON_L and LED0_OFF_L seperate
 *
 * @param[in]  num           The number
 * @param      dataReadOn0   The data read on 0
 * @param      dataReadOn1   The data read on 1
 * @param      dataReadOff0  The data read off 0
 * @param      dataReadOff1  The data read off 1
 *
 * @return     result of command
 */
esp_err_t getPWMDetail(uint8_t num, uint8_t* dataReadOn0, uint8_t* dataReadOn1, uint8_t* dataReadOff0, uint8_t* dataReadOff1)
{
    esp_err_t ret;

    uint8_t pinAddress = LED0_ON_L + LED_MULTIPLYER * num;

    ret = generic_read_two_i2c_register(pinAddress, dataReadOn0, dataReadOn1);
    if (ret != ESP_OK) {
        return ret;
    }

    pinAddress = LED0_OFF_L + LED_MULTIPLYER * num;
    ret = generic_read_two_i2c_register(pinAddress, dataReadOff0, dataReadOff1);

    return ret;
}

/**
 * @brief      Gets the pwm of a pin
 *
 * @param[in]  num      The number
 * @param      dataOn   The data on
 * @param      dataOff  The data off
 *
 * @return     result of command
 */
esp_err_t getPWM(uint8_t num, uint16_t* dataOn, uint16_t* dataOff)
{
    esp_err_t ret;

    uint8_t readPWMValueOn0;
    uint8_t readPWMValueOn1;
    uint8_t readPWMValueOff0;
    uint8_t readPWMValueOff1;

    ret = getPWMDetail(num, &readPWMValueOn0, &readPWMValueOn1, &readPWMValueOff0, &readPWMValueOff1);

    *dataOn = (readPWMValueOn1 << 8) | readPWMValueOn0;
    *dataOff = (readPWMValueOff1 << 8) | readPWMValueOff0;

    return ret;
}

/**
 * @brief      test function to show buffer
 *
 * @param      buf   The buffer
 * @param[in]  len   The length
 */
void disp_buf(uint16_t* buf, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++)
    {
        printf("%02x ", buf[i]);
        if (( i + 1 ) % 16 == 0)
        {
            printf("\n");
        }
    }
    printf("\n");
}

