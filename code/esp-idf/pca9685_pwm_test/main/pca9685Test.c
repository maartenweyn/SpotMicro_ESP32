#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "pca9685.h"

#include "sdkconfig.h"

#define I2C_EXAMPLE_MASTER_SCL_IO   22    /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO   23    /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_FREQ_HZ  100000     /*!< I2C master clock frequency */
#define I2C_EXAMPLE_MASTER_NUM      I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */


#define I2C_ADDRESS     0x40    /*!< lave address for PCA9685 */

#define ACK_CHECK_EN    0x1     /*!< I2C master will check ack from slave */
#define ACK_CHECK_DIS   0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL         0x0     /*!< I2C ack value */
#define NACK_VAL        0x1     /*!< I2C nack value */

static char tag[] = "PCA9685";

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

static void i2c_example_master_init(void);

/**
 * @brief i2c master initialization
 */
static void i2c_example_master_init(void)
{
    ESP_LOGD(tag, ">> PCA9685");
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0));
}

void servos_off()  {
     for (int i=0;i<12;i++)
        setPWM(i, 0, 0);
}

void task_PCA9685(void *ignore)
{
    printf("Executing on core %d\n", xPortGetCoreID());

    esp_err_t ret;

    i2c_example_master_init();

    set_pca9685_adress(I2C_ADDRESS);
    resetPCA9685();
    setFrequencyPCA9685(50); 

    printf("Finished setup, entering loop now\n");


    int16_t servo_min[12] = {153,118,138,  121,116,125,  131,150,148,  130,158,165}; //3% 
//    int16_t servo_min[12] = {153,138-20,138+0,  123-7+5,116,123-9+10,     123+8,123-50-0,123+10-8,    140-10,143+15,123+42}; //3% 

    int16_t servo_max[12] = {515,490-10,498+0,   450+35+5,470,490,   450+48,452,440,   505-10,490+15,462}; //12%
    //int16_t servo_min[12] = {163,153,151,121,126,124,131,73,125,143,185,140};

    float servo_conversion[12] = {1.927778,1.427778,1.705556,2.050000,2.050000,2.061111,2.038889,2.177778,1.650000,1.750000,2.233333,1.694444};

    int sleep_angle[12] = {90, 150, 0, 90, 30, 180, 90, 150, 0, 90, 30, 180};
    uint8_t from = 5;
    uint8_t to = from + 1;

    printf("const int16_t servo_min[12] = {");

    for (int i=0;i<12;i++)
            printf("%d,",servo_min[i]);

    printf("};\n\n");

    printf("const float servo_conversion[12] = {");

    for (int i=0;i<12;i++) {
        servo_conversion[i] = (servo_max[i]-servo_min[i])/180.0;
        printf("%f,",servo_conversion[i]);     
    }   

    printf("};\n\n");



    //servos_off();

    bool loop_sleep = true;

    while(loop_sleep) {
        for (int i=0;i<12;i++)
        {
            ret = setPWM(i, 0,  (uint16_t) (0.5 + servo_min[i] + (servo_conversion[i] * sleep_angle[i] )));
            printf("Servod %d: %f (%d)\n", i, servo_min[i] + (servo_conversion[i] * sleep_angle[i] ), (int) (0.5 +  servo_min[i] + (servo_conversion[i] * sleep_angle[i] )));
        }
        vTaskDelay(2000/portTICK_PERIOD_MS);
    };

    while(1)
    {

        //printf("Servos min\n");

        for (int i=from;i<to;i++) {
            ret = setPWM(i, 0, servo_min[i]);
            printf("Servod %d: %d (%d)\n", i, servo_min[i] , (int) (0.5 +  servo_min[i] ));
        }

        if(ret == ESP_ERR_TIMEOUT)
        {
            printf("I2C timeout\n");
        }
        else if(ret == ESP_OK)
        {
            // all good
        }
        else
        {
            printf("No ack, sensor not connected...skip...\n");
        }

        vTaskDelay(2000/portTICK_PERIOD_MS);

        //printf("Servos mid\n");

        for (int i=from;i<to;i++) {
            //ret = setPWM(i, 0, (servo_max[i] + servo_min[i]) / 2);
            //ret = setPWM(i, 0, servo_min[i] + (servo_max[i] - servo_min[i]) / 2);
            ret = setPWM(i, 0, (uint16_t) (0.5 + servo_min[i] + (servo_conversion[i] * 90 )));
            printf("Servod %d: %f (%d)\n", i, servo_min[i] + (servo_conversion[i] * 90 ), (int) (0.5 +  servo_min[i] + (servo_conversion[i] * 90 )));
        }
        

        vTaskDelay(1000 / portTICK_RATE_MS);

        //printf("Servos max\n");;

        for (int i=from;i<to;i++) {
            //ret = setPWM(i, 0, servo_max[i]);
            ret = setPWM(i, 0, (uint16_t) (0.5 + servo_min[i] + (servo_conversion[i] * 180 )));

            printf("Servod %d: %f (%d)\n", i, servo_min[i] + (servo_conversion[i] * 180 ), (int) (0.5 +  servo_min[i] + (servo_conversion[i] * 180 )));
        }
        
        vTaskDelay(2000 / portTICK_RATE_MS);

        //printf("Servos mid\n");

        for (int i=from;i<to;i++) {
            
            ret = setPWM(i, 0, (uint16_t) (0.5 + servo_min[i] + (servo_conversion[i] * 90 )));
            printf("Servod %d: %f (%d)\n", i, servo_min[i] + (servo_conversion[i] * 90 ), (int) (0.5 +  servo_min[i] + (servo_conversion[i] * 90 )));
        }
        

        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

void app_main()
{
    xTaskCreate(task_PCA9685, "task_PCA9685", 1024 * 2, (void* ) 0, 10, NULL);
}

