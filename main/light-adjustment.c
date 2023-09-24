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

#define GPIO_INPUT_IO_0     4
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0)

#define ESP_INTR_FLAG_DEFAULT 0

#define GPIO_OUTPUT_IO_0    18
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)

#define ON  1
#define OFF 0

#define I2C_MASTER_SCL_IO           26                         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           27                         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

// #define OPT3004_SENSOR_ADDR                     0x44                    /*!< Slave address of the OPT3004 sensor when ADDR connected to GND */
// #define OPT3004_RESULT_REG_ADDR                 0x00
// #define OPT3004_CONFIG_REG_ADDR                 0x01
// #define OPT3004_CONFIG_START_CONVERSION_BIT        9                    /*!< Bit to set for light sensor to start conversion process */
// #define OPT3004_CONFIG_CONVERSION_READY_BIT        7                    /*!< Bit is 1 when a conversion is ready (ready to read result register) - reading resets bit and INT pin */
// #define OPT3004_LOW_LIMIT_REG_ADDR              0x02
// #define OPT3004_LOW_LIMIT_EOC_MODE              0b1100 0000 0000 0000   /*!< Low level register value for end-of-conversion mode (interrupt every time conversion is complete)*/
// #define OPT3004_HIGH_LIMIT_REG_ADDR             0x03

//////////////////////////////////////////////////
// Triac trigger (digital output)
//////////////////////////////////////////////////
void trigger_setup(void) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

void trigger_enable(void) {
    gpio_set_level(GPIO_OUTPUT_IO_0, ON);
}

void trigger_disable(void) {
    gpio_set_level(GPIO_OUTPUT_IO_0, OFF);
}

//////////////////////////////////////////////////
// Zero crossing detection (digital input)
//////////////////////////////////////////////////
static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void zero_crossing_task(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "zero crossing detected");
            ////////////////////////////////////////////////// set alarm
        }
    }
}

void zero_crossing_setup(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(zero_crossing_task, "zero_crossing_task", 2048, NULL, 10, NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
}

//////////////////////////////////////////////////
// OPT3004 light sensor (I2C)
//////////////////////////////////////////////////
// static esp_err_t opt3004_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
// {
//     return i2c_master_write_read_device(I2C_MASTER_NUM, OPT3004_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
// }

// static esp_err_t opt3004_register_write(uint8_t reg_addr, uint8_t *data)
// {
//     int ret;
//     uint8_t write_buf[2] = {reg_addr, *data};

//     ret = i2c_master_write_to_device(I2C_MASTER_NUM, OPT3004_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

//     return ret;
// }

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

// static void opt3004_enable_conversion() {
//     data[2] = 1<<OPT3004_CONFIG_START_CONVERSION_BIT;
//     opt3004_register_write(OPT3004_CONFIG_REG_ADDR, data);
// }

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
    ESP_LOGI(TAG, "Create timer handle");
    light_measurement_alarm_queue = xQueueCreate(10, sizeof(light_measurement_alarm_queue));
    if (!light_measurement_alarm_queue) {
        ESP_LOGE(TAG, "Creating queue failed");
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

    ESP_LOGI(TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    ESP_LOGI(TAG, "Start timer, auto-reload at alarm event");
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 5000000, // period = 5s
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

//////////////////////////////////////////////////
//
//////////////////////////////////////////////////
void app_main() {
    trigger_setup();
    zero_crossing_setup();

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    light_measurement_timer_setup();

    // int cnt = 0;
    for (;;) {
        // printf("cnt: %d\n", cnt++);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        trigger_enable();
        vTaskDelay(250 / portTICK_PERIOD_MS);
        trigger_disable();
    }

    // opt3004_enable_conversion();


    // enable timers
    // - timer every x ms to update light sensor reading
    //   - compare actual light with target light
    //   - if not within tolerance, update phase delay
    // - timer every 8.3 ms to simulate zero cross detection
    //   - enable phase delay timer and interrupt
    // - phase delay timer (enabled by zero cross)
    //   - send pulse to triac
    // interrupt on opt3004 INT pin - tells when to read value (interrupt on LOW / negedge)
    // - how do I reset INT pin?
    //   - need to read config ready register
    // AC 8.3 ms, light sensor 800 ms
    // may need to work on interrupt priority in future
}


// GPIO example:                        https://github.com/espressif/esp-idf/blob/v5.1/examples/peripherals/gpio/generic_gpio/README.md
// GPIO documentation:                  https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/peripherals/gpio.html

// I2C example:                         https://github.com/espressif/esp-idf/blob/v5.1/examples/peripherals/i2c/i2c_simple/README.md
// I2C documentation:                   https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/peripherals/i2c.html

// General purpose timer example:       https://github.com/espressif/esp-idf/tree/v5.1/examples/peripherals/timer_group/gptimer
// General purpose timer documentation: https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/peripherals/timer.html#_CPPv421timer_set_alarm_value13timer_group_t11timer_idx_t8uint64_t
