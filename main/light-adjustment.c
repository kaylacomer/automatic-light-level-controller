//////////////////////////////////////////////////
// Header files
//////////////////////////////////////////////////
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include "driver/timer.h"

//////////////////////////////////////////////////
// Pin mapping and global constants
//////////////////////////////////////////////////
#define CONFIG_FREERTOS_HZ 1000
#define TICK_WAIT_PERIOD 100
#define CONFIG_COMPILER_OPTIMIZATION -O2

static const char *TAG = "light-adjustment";

#define ESP_INTR_FLAG_DEFAULT 0

#define GPIO_INPUT_ZERO_CROSS_PIN     2
#define ZERO_CROSSING_PIN_SEL  (1ULL<<GPIO_INPUT_ZERO_CROSS_PIN)

#define GPIO_INPUT_LTR303_INTR_PIN     5
#define LTR303_INTR_PIN_SEL (1ULL<<GPIO_INPUT_LTR303_INTR_PIN)

#define GPIO_OUTPUT_TRIAC_TRIGGER_PIN    18
#define GPIO_OUTPUT_TRIAC_TRIGGER_PIN_SEL  (1ULL<<GPIO_OUTPUT_TRIAC_TRIGGER_PIN)

#define ON  1
#define OFF 0

#define I2C_MASTER_SCL_IO           26                         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           27                         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define LTR303_SENSOR_ADDR          0x29
#define LTR303_REG_CONTR            0x80  // operation mode control SW reset
#define LTR303_REG_CONTR_MODE_BIT      0  // bit to toggle active mode
#define LTR303_REG_CONTR_4XGAIN_BIT    3  // bit to enable 4X gain -> measurement ranges 0.25 to 16 klux
#define LTR303_REG_CONTR_2XGAIN_BIT    2  // bit to enable 2X gain -> measurement ranges 0.5 to 32 klux
#define LTR303_REG_MEAS_RATE        0x85  // measurement rate in active mode
#define LTR303_REG_MEAS_RATE_BIT       2  // set measurement rate to 2 s
#define LTR303_REG_DATA_CH1_0       0x88  // CH1 data lower byte
#define LTR303_REG_DATA_CH1_1       0x89  //           upper
#define LTR303_REG_DATA_CH0_0       0x8A  // CH0 data lower byte
#define LTR303_REG_DATA_CH0_1       0x8B  //           upper
#define LTR303_REG_STATUS           0x8C  // new data status
#define LTR303_REG_INTERRUPT        0x8F  // interrupt settings
#define LTR303_REG_INTR_MODE_BIT       1  // bit to toggle interrupt mode (with logic 0 indicating interrupt)
#define LTR303_REG_THRES_UP_0       0x97  // upper limit of interrupt threshold value, lower byte - interrupt is called if measurement is outside of range
#define LTR303_REG_THRES_UP_1       0x98  // upper                                   , upper
#define LTR303_REG_THRES_LOW_0      0x98  // lower                                   , lower
#define LTR303_REG_THRES_LOW_1      0x98  // lower                                   , upper
#define LTR303_THRES_LOW_INTR_VAL   0xFF  // value for lower bits so that interrupt is called on every measurement

uint16_t phase_delay_time = 100;
bool phase_delay_timer_on = false;
gptimer_handle_t phase_delay_timer = NULL;
gptimer_alarm_config_t phase_delay_timer_alarm_config = {
    .alarm_count = 0,
    .flags.auto_reload_on_alarm = false,
};

uint16_t light_meas_lux = 0;
uint16_t target_val_lux = 0;

bool update_alarm_val = true;


//////////////////////////////////////////////////
// Triac trigger (digital output)
//////////////////////////////////////////////////
void trigger_setup(void) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_TRIAC_TRIGGER_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

void trigger_enable(void) {
    gpio_set_level(GPIO_OUTPUT_TRIAC_TRIGGER_PIN, ON);
}

void trigger_disable(void) {
    gpio_set_level(GPIO_OUTPUT_TRIAC_TRIGGER_PIN, OFF);
}

//////////////////////////////////////////////////
// LTR303 light sensor (I2C)
//////////////////////////////////////////////////
static esp_err_t ltr303_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, LTR303_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t ltr303_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, LTR303_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

static void ltr303_setup() {
    uint8_t data[1];

    // set lower threshold register to max value to trigger interrupt on every data read
    ltr303_register_write_byte(LTR303_REG_THRES_LOW_0, LTR303_THRES_LOW_INTR_VAL);
    ltr303_register_write_byte(LTR303_REG_THRES_LOW_1, LTR303_THRES_LOW_INTR_VAL);

    // set measurement rate to every 2 seconds
    ltr303_register_write_byte(LTR303_REG_MEAS_RATE, 1<<LTR303_REG_MEAS_RATE_BIT);
    
    ltr303_register_read(LTR303_REG_MEAS_RATE, data, 1);
    ESP_LOGI(TAG, "meas rate: %d", LTR303_REG_MEAS_RATE);

    // set measurement mode to active
    ltr303_register_write_byte(LTR303_REG_CONTR, 1<<LTR303_REG_CONTR_MODE_BIT | 1<<LTR303_REG_CONTR_2XGAIN_BIT);
}

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

//////////////////////////////////////////////////
// Zero crossing detection (digital input)
//////////////////////////////////////////////////
static void IRAM_ATTR zero_crossing_isr_handler(void* arg)
{
    trigger_disable();
    if (phase_delay_timer_on) {
        gptimer_stop(phase_delay_timer);
    }
    phase_delay_timer_on = true;
    gptimer_set_raw_count(phase_delay_timer, 0);

    if (update_alarm_val) {
        phase_delay_timer_alarm_config.alarm_count = phase_delay_time;
        gptimer_set_alarm_action(phase_delay_timer, &phase_delay_timer_alarm_config);
        update_alarm_val = false;
    }

    gptimer_start(phase_delay_timer);
}

void zero_crossing_setup(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = ZERO_CROSSING_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_ZERO_CROSS_PIN, zero_crossing_isr_handler, NULL);
}

//////////////////////////////////////////////////
// Phase delay timer
//////////////////////////////////////////////////
static bool IRAM_ATTR phase_delay_timer_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    trigger_enable();
    BaseType_t high_task_awoken = pdTRUE;

    // // return whether we need to yield at the end of ISR
    return (high_task_awoken);
}

static void phase_delay_timer_setup()
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &phase_delay_timer));

    gptimer_event_callbacks_t phase_delay_timer_callback = {
        .on_alarm = phase_delay_timer_handler,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(phase_delay_timer, &phase_delay_timer_callback, NULL));

    ESP_ERROR_CHECK(gptimer_enable(phase_delay_timer));
}

//////////////////////////////////////////////////
//
//////////////////////////////////////////////////
void app_main() {
    trigger_setup();

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    ltr303_setup();

    phase_delay_timer_setup();

    zero_crossing_setup();

    target_val_lux = 15000;
    uint16_t tolerance_lux = 100;
    uint8_t delay_seconds = 3;

    while(1) {
        vTaskDelay(delay_seconds * 100);

        uint8_t visible_and_infrared[2];
        ltr303_register_read(LTR303_REG_DATA_CH1_0, visible_and_infrared, 2); // ch 1 - visible and infrared
        uint8_t infrared[2];
        ltr303_register_read(LTR303_REG_DATA_CH0_0, infrared, 2); // ch 0 - infrared

        uint16_t light_meas_lux = visible_and_infrared[1]<<8 | visible_and_infrared[0];
        ESP_LOGI(TAG, "light measurement: %d lux", light_meas_lux);
        
        if (abs(target_val_lux - light_meas_lux) > tolerance_lux) { // if alarm val will be updated, set flag to true
            update_alarm_val = true;
        }
        
        if (abs(target_val_lux - light_meas_lux) > 10*tolerance_lux) {
            if (target_val_lux > light_meas_lux) {
                phase_delay_time -= 1000;
            }
            else {
                phase_delay_time += 1000;
            }
            delay_seconds = 3;
        }
        else if (abs(target_val_lux - light_meas_lux) > 3*tolerance_lux) {
            if (target_val_lux > light_meas_lux) {
                phase_delay_time -= 200;
            }
            else {
                phase_delay_time += 200;
            }
            delay_seconds = 7;
        }
        else if (abs(target_val_lux - light_meas_lux) > tolerance_lux) {
            if (target_val_lux > light_meas_lux) {
                phase_delay_time -= 100;
            }
            else {
                phase_delay_time += 100;
            }
            delay_seconds = 15;
        }
        
        if (phase_delay_time > 50000) { // if phase_delay_time decreased and overflowed to very high value, reset to minimum time
            phase_delay_time = 100;
        }
        else if (phase_delay_time > 8500) { // if phase_delay_time increased above max, set to 10000 ms (light effectively turned off)
            phase_delay_time = 10000;
        }

        ESP_LOGI(TAG, "phase delay time: %d", phase_delay_time);
    }
}
// integrate with WiFi / mesh network


// GPIO example:                        https://github.com/espressif/esp-idf/blob/v5.1/examples/peripherals/gpio/generic_gpio/
// GPIO documentation:                  https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/peripherals/gpio.html

// I2C example:                         https://github.com/espressif/esp-idf/blob/v5.1/examples/peripherals/i2c/i2c_simple/
// I2C documentation:                   https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/peripherals/i2c.html

// General purpose timer example:       https://github.com/espressif/esp-idf/tree/v5.1/examples/peripherals/timer_group/gptimer
// General purpose timer documentation: https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/peripherals/timer.html#_CPPv421timer_set_alarm_value13timer_group_t11timer_idx_t8uint64_t
