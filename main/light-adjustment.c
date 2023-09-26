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

//////////////////////////////////////////////////
// Pin mapping and global constants
//////////////////////////////////////////////////
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
#define LTR303_REG_MEAS_RATE        0x85  // measurement rate in active mode
#define LTR303_REG_DATA_CH1_0       0x88  // CH1 data lower byte
#define LTR303_REG_DATA_CH1_1       0x89  //           upper
#define LTR303_REG_DATA_CH0_0       0x8A  // CH0 data lower byte
#define LTR303_REG_DATA_CH0_1       0x8B  //           upper
#define LTR303_REG_STATUS           0x8C  // new data status
#define LTR303_REG_INTERRUPT        0x8F  // interrupt settings
#define LTR303_REG_INTR_MODE_BIT       1  // bit to toggle interrupt mode (logic 0 indicates interrupt)
#define LTR303_REG_THRES_UP_0       0x97  // upper limit of interrupt threshold value, lower byte - interrupt is called if measurement is outside of range
#define LTR303_REG_THRES_UP_1       0x98  // upper                                   , upper
#define LTR303_REG_THRES_LOW_0      0x98  // lower                                   , lower
#define LTR303_REG_THRES_LOW_1      0x98  // lower                                   , upper
#define LTR303_THRES_LOW_INTR_VAL   0xFF  // value for lower bits so that interrupt is called on every measurement

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

static QueueHandle_t ltr303_meas_evt_queue = NULL;

static void IRAM_ATTR ltr303_meas_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(ltr303_meas_evt_queue, &gpio_num, NULL);
}

static void ltr303_meas_task(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(ltr303_meas_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "new data ready to be read from ltr303");
            ///// read data from ltr 303
            ltr303_register_write_byte(LTR303_REG_CONTR, 1<<LTR303_REG_CONTR_MODE_BIT);  // turn on standby mode to save power
        }
    }
}

static void ltr303_setup() {
    // enable interrupt mode
    ltr303_register_write_byte(LTR303_REG_INTERRUPT, 1<<LTR303_REG_INTR_MODE_BIT);

    // set lower threshold register to max value to trigger interrupt on every data read
    ltr303_register_write_byte(LTR303_REG_THRES_LOW_0, LTR303_THRES_LOW_INTR_VAL);
    ltr303_register_write_byte(LTR303_REG_THRES_LOW_1, LTR303_THRES_LOW_INTR_VAL);

    // create interrupt on gpio pin connected to ltr303 INT
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = LTR303_INTR_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    ltr303_meas_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(ltr303_meas_task, "ltr303_meas_task", 2048, NULL, 10, NULL);

    gpio_isr_handler_add(GPIO_INPUT_LTR303_INTR_PIN, ltr303_meas_isr_handler, (void*) GPIO_INPUT_LTR303_INTR_PIN);
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
// Light sensor reading timer
//////////////////////////////////////////////////
static QueueHandle_t light_measurement_alarm_queue = NULL;

static void light_measurement_task(void* arg)
{
    uint32_t user_data;
    for(;;) {
        if (xQueueReceive(light_measurement_alarm_queue, &user_data, pdMS_TO_TICKS(2000))) {
            ESP_LOGI(TAG, "enable light measurement conversion (read light level)");
            ltr303_register_write_byte(LTR303_REG_CONTR, 1<<LTR303_REG_CONTR_MODE_BIT);
        }
    }
}

static bool IRAM_ATTR light_measurement_timer_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    xQueueSendFromISR(light_measurement_alarm_queue, &user_data, &high_task_awoken);

    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}

static void light_measurement_timer_setup() {
    ESP_LOGI(TAG, "Create light measurement timer handle");
    light_measurement_alarm_queue = xQueueCreate(10, sizeof(light_measurement_alarm_queue));
    if (!light_measurement_alarm_queue) {
        ESP_LOGE(TAG, "Creating light measurement queue failed");
        return;
    }
    xTaskCreate(light_measurement_task, "light_measurement_task", 2048, NULL, 10, NULL);

    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t light_measurement_timer_callback = {
        .on_alarm = light_measurement_timer_handler,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &light_measurement_timer_callback, light_measurement_alarm_queue));

    ESP_LOGI(TAG, "Enable light measurement timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    ESP_LOGI(TAG, "Start light measurement timer, auto-reload at alarm event");
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 1000000, // period = 1s
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

//////////////////////////////////////////////////
// Triac trigger after phase delay timer
//////////////////////////////////////////////////
static QueueHandle_t phase_delay_alarm_queue = NULL;
gptimer_handle_t phase_delay_timer = NULL;

static void phase_delay_task(void* arg)
{
    ESP_LOGI(TAG, "in phase_delay_task");

    uint32_t user_data;
    for(;;) {
        if (xQueueReceive(phase_delay_alarm_queue, &user_data, pdMS_TO_TICKS(2000))) {
            ESP_LOGI(TAG, "trigger triac after phase delay");
        }
    }
}

static bool IRAM_ATTR phase_delay_timer_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    ESP_LOGI(TAG, "in phase delay timer handler");
    BaseType_t high_task_awoken = pdTRUE;

    ESP_ERROR_CHECK(gptimer_stop(phase_delay_timer));
    
    // ESP_ERROR_CHECK(gptimer_disable(phase_delay_timer));
    
    xQueueSendFromISR(phase_delay_alarm_queue, &user_data, &high_task_awoken);

    // return whether we need to yield at the end of ISR
    return (high_task_awoken);
}

static void phase_delay_timer_setup()
{
    ESP_LOGI(TAG, "Create phase delay timer handler");
    phase_delay_alarm_queue = xQueueCreate(10, sizeof(phase_delay_alarm_queue));
    if (!phase_delay_alarm_queue) {
        ESP_LOGE(TAG, "Creating phase delay alarm failed");
        return;
    }
    xTaskCreate(phase_delay_task, "phase_delay_task", 2048, NULL, 10, NULL);

    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
        .intr_priority = 3,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &phase_delay_timer));

    gptimer_event_callbacks_t phase_delay_timer_callback = {
        .on_alarm = phase_delay_timer_handler,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(phase_delay_timer, &phase_delay_timer_callback, phase_delay_alarm_queue));

    ESP_LOGI(TAG, "Enable phase delay timer");
    ESP_ERROR_CHECK(gptimer_enable(phase_delay_timer));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 10000, // period = 0.01s
        .flags.auto_reload_on_alarm = false,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(phase_delay_timer, &alarm_config));
}

static void phase_delay_timer_start() {
    ESP_LOGI(TAG, "Start phase delay timer, no auto-reload");
    
    ESP_ERROR_CHECK(gptimer_set_raw_count(phase_delay_timer, 0));
    ESP_ERROR_CHECK(gptimer_start(phase_delay_timer));
}

//////////////////////////////////////////////////
// Zero crossing detection (digital input)
//////////////////////////////////////////////////
static QueueHandle_t zero_crossing_evt_queue = NULL;

static void IRAM_ATTR zero_crossing_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(zero_crossing_evt_queue, &gpio_num, NULL);
}

static void zero_crossing_task(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(zero_crossing_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "zero crossing detected - start phase delay timer");
            phase_delay_timer_start();
        }
    }
}

void zero_crossing_setup(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = ZERO_CROSSING_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    // io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    zero_crossing_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(zero_crossing_task, "zero_crossing_task", 2048, NULL, 10, NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_ZERO_CROSS_PIN, zero_crossing_isr_handler, (void*) GPIO_INPUT_ZERO_CROSS_PIN);
}

//////////////////////////////////////////////////
//
//////////////////////////////////////////////////
void app_main() {
    trigger_setup();

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    light_measurement_timer_setup();
    phase_delay_timer_setup();

    zero_crossing_setup();

    // ltr303_setup();

    // int cnt = 0;
    for (;;) {
        // printf("cnt: %d\n", cnt++);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        trigger_enable();
        vTaskDelay(250 / portTICK_PERIOD_MS);
        trigger_disable();
    }


    // enable timers
    // - timer every x ms to update light sensor reading
    //   - compare actual light with target light
    //   - if not within tolerance, update phase delay
    // - 120 hz square wave to simulate zero cross detection
    //   - enable phase delay timer and interrupt
    // - phase delay timer (enabled by zero cross)
    //   - send pulse to triac
    // interrupt on ltr303 INT pin - tells when to read value (interrupt on LOW / negedge)
    // - how do I reset INT pin?
    //   - auto reset by reading register
    // AC 8.3 ms, light sensor 100 ms integration
    // may need to work on interrupt priority in future
    // can change light sensor gain to smaller range of lux
}


// GPIO example:                        https://github.com/espressif/esp-idf/blob/v5.1/examples/peripherals/gpio/generic_gpio/
// GPIO documentation:                  https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/peripherals/gpio.html

// I2C example:                         https://github.com/espressif/esp-idf/blob/v5.1/examples/peripherals/i2c/i2c_simple/
// I2C documentation:                   https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/peripherals/i2c.html

// General purpose timer example:       https://github.com/espressif/esp-idf/tree/v5.1/examples/peripherals/timer_group/gptimer
// General purpose timer documentation: https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/peripherals/timer.html#_CPPv421timer_set_alarm_value13timer_group_t11timer_idx_t8uint64_t
