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
// #define portTICK_PERIOD_MS 1 // ( ( TickType_t ) 1000 / CONFIG_FREERTOS_HZ )
#define TICK_WAIT_PERIOD 100
#define CONFIG_COMPILER_OPTIMIZATION -O2
// #define configCHECK_FOR_STACK_OVERFLOW 2

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

bool phase_delay_timer_on = false;
// bool zero_crossing_detected = false;
gptimer_handle_t phase_delay_timer = NULL;

// #define GPIO_SET_LEVEL_HIGH(gpio_num) (GPIO.out_w1ts = (1 << gpio_num))
// #define GPIO_SET_LEVEL_LOW(gpio_num) (GPIO.out_w1tc = (1 << gpio_num))

uint64_t timer_val = 0;
int64_t zero_cross_time = 0;
int64_t phase_delay_time = 0;
uint16_t light_meas = 0;


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
    // GPIO_SET_LEVEL_HIGH(GPIO_OUTPUT_TRIAC_TRIGGER_PIN); // Sets GPIO 18 HIGH
    gpio_set_level(GPIO_OUTPUT_TRIAC_TRIGGER_PIN, ON);
}

void trigger_disable(void) {
    // GPIO_SET_LEVEL_LOW(GPIO_OUTPUT_TRIAC_TRIGGER_PIN);  // Sets GPIO 18 LOW
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
        if(xQueueReceive(ltr303_meas_evt_queue, &io_num, TICK_WAIT_PERIOD)) {
            ESP_LOGI(TAG, "new data ready to be read from ltr303");
            // ltr303_register_write_byte(LTR303_REG_CONTR, 0);  // turn on standby mode to save power
            // uint8_t data[1];
            uint8_t visible_and_infrared[2];
            uint8_t infrared[2];
            ltr303_register_read(LTR303_REG_DATA_CH1_0, visible_and_infrared, 2); // ch 1 lower byte
            // ltr303_register_read(LTR303_REG_DATA_CH1_1, data, 1); // upper
            ltr303_register_read(LTR303_REG_DATA_CH0_0, infrared, 2); // ch 0 lower byte
            // ltr303_register_read(LTR303_REG_DATA_CH0_1, data, 1); // upper
            light_meas = (8<<(visible_and_infrared[1] - infrared[1])) | (visible_and_infrared[0] - infrared[0]);
            // light_meas[1] = visible_and_infrared[1] - infrared[1];
            // light_meas[0] = visible_and_infrared[0] - infrared[0];
            ESP_LOGI(TAG, "visible light: %d", light_meas);
        }
    }
}

static void ltr303_setup() {
    uint8_t data[1];

    // set lower threshold register to max value to trigger interrupt on every data read
    ltr303_register_write_byte(LTR303_REG_THRES_LOW_0, LTR303_THRES_LOW_INTR_VAL);
    ltr303_register_write_byte(LTR303_REG_THRES_LOW_1, LTR303_THRES_LOW_INTR_VAL);

    // enable interrupt mode
    ltr303_register_write_byte(LTR303_REG_INTERRUPT, 1<<LTR303_REG_INTR_MODE_BIT);

    // set measurement rate to every 2 seconds
    // ltr303_register_write_byte(LTR303_REG_MEAS_RATE, 1<<LTR303_REG_MEAS_RATE_BIT);
    
    ltr303_register_read(LTR303_REG_MEAS_RATE, data, 1);
    ESP_LOGI(TAG, "meas rate: %d", LTR303_REG_MEAS_RATE);

    // set measurement mode to active
    ltr303_register_write_byte(LTR303_REG_CONTR, 1<<LTR303_REG_CONTR_MODE_BIT);

    // create interrupt on gpio pin connected to ltr303 INT
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = LTR303_INTR_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    ltr303_meas_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(ltr303_meas_task, "ltr303_meas_task", 2048, NULL, 10, NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
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

// static QueueHandle_t light_measurement_alarm_queue = NULL;

// static void light_measurement_task(void* arg)
// {
//     uint32_t user_data;
//     for(;;) {
//         if (xQueueReceive(light_measurement_alarm_queue, &user_data, TICK_WAIT_PERIOD)) {
//             ESP_LOGI(TAG, "enable light measurement conversion (read light level)");
//             ltr303_register_write_byte(LTR303_REG_INTERRUPT, 1<<LTR303_REG_INTR_MODE_BIT);  // enable interrupt on ltr303 when conversion complete
//             ltr303_register_write_byte(LTR303_REG_MEAS_RATE, LTR303_REG_MEAS_RATE_BITS); // set measurement rate to every 2 seconds
//             ltr303_register_write_byte(LTR303_REG_CONTR, 1<<LTR303_REG_CONTR_MODE_BIT);  // set measurement mode to active
//         }
//     }
// }

// static bool IRAM_ATTR light_measurement_timer_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
// {
//     BaseType_t high_task_awoken = pdFALSE;
//     xQueueSendFromISR(light_measurement_alarm_queue, &user_data, &high_task_awoken);

//     // return whether we need to yield at the end of ISR
//     return (high_task_awoken == pdTRUE);
// }

// gptimer_handle_t light_measurement_timer = NULL;
// static void light_measurement_timer_setup() {
//     ESP_LOGI(TAG, "Create light measurement timer handle");
//     light_measurement_alarm_queue = xQueueCreate(10, sizeof(light_measurement_alarm_queue));
//     if (!light_measurement_alarm_queue) {
//         ESP_LOGE(TAG, "Creating light measurement queue failed");
//         return;
//     }
//     xTaskCreate(light_measurement_task, "light_measurement_task", 2048, NULL, 10, NULL);

//     gptimer_config_t timer_config = {
//         .clk_src = GPTIMER_CLK_SRC_DEFAULT,
//         .direction = GPTIMER_COUNT_UP,
//         .resolution_hz = 1000000, // 1MHz, 1 tick=1us
//     };
//     ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &light_measurement_timer));

//     gptimer_event_callbacks_t light_measurement_timer_callback = {
//         .on_alarm = light_measurement_timer_handler,
//     };
//     ESP_ERROR_CHECK(gptimer_register_event_callbacks(light_measurement_timer, &light_measurement_timer_callback, light_measurement_alarm_queue));

//     ESP_LOGI(TAG, "Enable light measurement timer");
//     ESP_ERROR_CHECK(gptimer_enable(light_measurement_timer));

//     ESP_LOGI(TAG, "Start light measurement timer, auto-reload at alarm event");
//     gptimer_alarm_config_t alarm_config = {
//         .reload_count = 0,
//         .alarm_count = 5000000, // period = 5s
//         .flags.auto_reload_on_alarm = true,
//     };
//     ESP_ERROR_CHECK(gptimer_set_alarm_action(light_measurement_timer, &alarm_config));
//     ESP_ERROR_CHECK(gptimer_start(light_measurement_timer));
// }

//////////////////////////////////////////////////
// Zero crossing detection (digital input)
//////////////////////////////////////////////////
// static QueueHandle_t zero_crossing_evt_queue = NULL;
// int64_t zero_crossing_isr_task_time = 0;

static void IRAM_ATTR zero_crossing_isr_handler(void* arg)
{
    // zero_crossing_isr_task_time = esp_timer_get_time();
    // xQueueSendFromISR(zero_crossing_evt_queue, &zero_crossing_isr_task_time, NULL);
    // ESP_LOGI(TAG, "zero_crossing_isr_handler");
    trigger_disable();
    gptimer_get_raw_count(phase_delay_timer, &timer_val);
    if (phase_delay_timer_on) {
        gptimer_stop(phase_delay_timer);
    }
    phase_delay_timer_on = true;
    gptimer_set_raw_count(phase_delay_timer, 0);
    gptimer_start(phase_delay_timer);
}

// #define DEBOUNCE_TIME_MS 2 // Adjust based on your needs

// static uint32_t last_zero_crossing_time = 0;
// static bool last_zero_crossing_state = false;
// // bool phase_delay_timer_on = false;

// static void zero_crossing_task(void* arg)
// {
//     for (;;) {
//         // int64_t time_of_zero_crossing;
//         if(xQueueReceive(zero_crossing_evt_queue, &zero_crossing_isr_task_time, TICK_WAIT_PERIOD)) {
//             trigger_disable();
//             if (phase_delay_timer_on) {
//                 gptimer_stop(phase_delay_timer);
//             }
//             phase_delay_timer_on = true;
//             gptimer_set_raw_count(phase_delay_timer, 0);
//             gptimer_start(phase_delay_timer);
//     //                 // gptimer_get_raw_count(phase_delay_timer, &timer_val);
//             // gptimer_start_once(phase_delay_timer, 4000);
//             // timer_pause(TIMER_GROUP_0, TIMER_0);
//             // timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0xULL);
//             // timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 4000);
//             // timer_enable_alarm(TIMER_GROUP_0, TIMER_0);
//             // timer_start(TIMER_GROUP_0, TIMER_0);
//         }
//     }
// }
    // for(;;) {
    //     if(xQueueReceive(zero_crossing_evt_queue, &io_num, TICK_WAIT_PERIOD)) {

    //         zero_crossing_isr_task_time = esp_timer_get_time();
    //         uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS; // Convert ticks to milliseconds

    //         bool current_state = gpio_get_level(GPIO_INPUT_ZERO_CROSS_PIN); // Get the current state

    //         if (current_state != last_zero_crossing_state) {
    //             if ((current_time - last_zero_crossing_time) >= DEBOUNCE_TIME_MS) {
    //                 // Acknowledge the change
    //                 last_zero_crossing_state = current_state;
    //                 last_zero_crossing_time = current_time;
                    
    //                 // Rest of the code to handle zero crossing
    //                 trigger_disable();

    //                 if (phase_delay_timer_on) {
    //                     gptimer_stop(phase_delay_timer);
    //                 }
    //                 gptimer_get_raw_count(phase_delay_timer, &timer_val);
    //                 phase_delay_timer_on = true;
    //                 gptimer_set_raw_count(phase_delay_timer, 0);
    //                 // gptimer_get_raw_count(phase_delay_timer, &timer_val);
    //                 gptimer_start(phase_delay_timer);

    //                 zero_crossing_isr_task_time = esp_timer_get_time() - zero_crossing_isr_task_time;
    //             }
    //         }
    //     }
    // }
// }


// static void zero_crossing_task(void* arg)
// {
//     uint32_t io_num;
//     for(;;) {
//         if(xQueueReceive(zero_crossing_evt_queue, &io_num, TICK_WAIT_PERIOD)) {
//             zero_cross_time = esp_timer_get_time();

//             trigger_disable();

//             if (phase_delay_timer_on) {
//                 ESP_ERROR_CHECK(gptimer_stop(phase_delay_timer));
//             }
//             gptimer_get_raw_count(phase_delay_timer, &timer_val);
//             phase_delay_timer_on = true;
//             ESP_ERROR_CHECK(gptimer_set_raw_count(phase_delay_timer, 0));
//             ESP_ERROR_CHECK(gptimer_start(phase_delay_timer));
//         }
//     }
// }

void zero_crossing_setup(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = ZERO_CROSSING_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // zero_crossing_evt_queue = xQueueCreate(10, sizeof(int64_t));
    // xTaskCreate(zero_crossing_task, "zero_crossing_task", 2048, NULL, configMAX_PRIORITIES, NULL);

    // gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_ZERO_CROSS_PIN, zero_crossing_isr_handler, NULL);
}

//////////////////////////////////////////////////
// Phase delay timer
//////////////////////////////////////////////////
// static QueueHandle_t phase_delay_alarm_queue = NULL;
// int64_t phase_delay_timer_isr_task_time = 0;

// static void phase_delay_task(void* arg)
// {
//     uint32_t user_data;
//     for(;;) {
//         if (xQueueReceive(phase_delay_alarm_queue, &user_data, TICK_WAIT_PERIOD)) {
//             phase_delay_time = esp_timer_get_time();
//             // gptimer_get_raw_count(phase_delay_timer, &timer_val);
//             // ESP_LOGI(TAG, "phase delay alarm task, %ld", xTaskGetTickCount());
//             // int64_t zero_cross_time = esp_timer_get_time();
//             // ESP_LOGI(TAG, "Current time: %lld microseconds", zero_cross_time);
//             trigger_enable();

//             phase_delay_timer_isr_task_time = esp_timer_get_time() - phase_delay_timer_isr_task_time;
//             // zero_crossing_detected = false;
//             // gpio_isr_handler_add(GPIO_INPUT_ZERO_CROSS_PIN, zero_crossing_isr_handler, (void*) GPIO_INPUT_ZERO_CROSS_PIN);
//         }
//     }
// }

static bool IRAM_ATTR phase_delay_timer_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    // ESP_LOGI(TAG, "phase_delay_timer_handler");
    trigger_enable();
    // return true;
    BaseType_t high_task_awoken = pdTRUE;
    // phase_delay_timer_isr_task_time = esp_timer_get_time();
    
    // xQueueSendFromISR(phase_delay_alarm_queue, &user_data, &high_task_awoken);

    // // return whether we need to yield at the end of ISR
    return (high_task_awoken);
}

static void phase_delay_timer_setup()
{
    // ESP_LOGI(TAG, "Create phase delay timer handler");
    // phase_delay_alarm_queue = xQueueCreate(1024, sizeof(phase_delay_alarm_queue));
    // if (!phase_delay_alarm_queue) {
    //     ESP_LOGE(TAG, "Creating phase delay alarm failed");
    //     return;
    // }
    // xTaskCreate(phase_delay_task, "phase_delay_task", 1024, NULL, configMAX_PRIORITIES-1, NULL);

    gptimer_config_t timer_config = {
        // .divider = 80,
        // .count_up = false,
        // .alarm_enable = true,
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
        // .intr_priority = 3,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &phase_delay_timer));

    gptimer_event_callbacks_t phase_delay_timer_callback = {
        .on_alarm = phase_delay_timer_handler,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(phase_delay_timer, &phase_delay_timer_callback, NULL));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 16000, // period = x ms
        .flags.auto_reload_on_alarm = false,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(phase_delay_timer, &alarm_config));

    ESP_ERROR_CHECK(gptimer_enable(phase_delay_timer));
    // gptimer_init(TIMER_GROUP_O, TIMER_0, &timer_config);
    // gptimer_isr_register(TIMER_GROUP_0, TIMER_0, phase_delay_timer_handler, NULL, ESP_INTR_FLAG_IRAM, &phase_delay_timer);
    // gptimer_enable_alarm(TIMER_GROUP_O, TIMER_0);
}

//////////////////////////////////////////////////
//
//////////////////////////////////////////////////
void app_main() {
    trigger_setup();

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    ltr303_setup();

    // light_measurement_timer_setup();
    phase_delay_timer_setup();

    zero_crossing_setup();

    while(1) {
        // uint32_t heap_size = 0;
        // heap_size = esp_get_free_heap_size();
        // ESP_LOGI(TAG, "Heap size: %ld", heap_size);
        // ESP_LOGI(TAG, "Timer val: %lld", timer_val);
        // ESP_LOGI(TAG, "Zero cross time: %lld microseconds", zero_cross_time);
        // ESP_LOGI(TAG, "Phase delay time: %lld microseconds", phase_delay_time);
        // ESP_LOGI(TAG, "Zero crossing isr and task time: %lld microseconds", zero_crossing_isr_task_time);
        // ESP_LOGI(TAG, "Phase delay isr and task time: %lld microseconds", phase_delay_timer_isr_task_time);
        vTaskDelay(1000);

        uint8_t visible_and_infrared[2];
        uint8_t infrared[2];
        ltr303_register_read(LTR303_REG_DATA_CH1_0, visible_and_infrared, 2); // ch 1 lower byte
        uint16_t total_light = visible_and_infrared[0] | 8<<visible_and_infrared[1];
        ESP_LOGI(TAG, "visible and infrared light: %d", total_light);
        // ltr303_register_read(LTR303_REG_DATA_CH1_1, data, 1); // upper
        ltr303_register_read(LTR303_REG_DATA_CH0_0, infrared, 2); // ch 0 lower byte
        uint16_t infrared_total = infrared[0] | 8<<infrared[1];
        // ESP_LOGI(TAG, "infrared light: %d", infrared_total);
        // ltr303_register_read(LTR303_REG_DATA_CH0_1, data, 1); // upper
        // light_meas = (8<<(visible_and_infrared[1] - infrared[1])) | (visible_and_infrared[0] - infrared[0]);
        // ESP_LOGI(TAG, "light measurement: %d", light_meas);
        // light_meas[1] = visible_and_infrared[1] - infrared[1];
        // light_meas[0] = visible_and_infrared[0] - infrared[0];
        

    }

    // uint8_t data[1];
    // ltr303_register_read(LTR303_REG_DATA_CH1_0, data, 1);


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
