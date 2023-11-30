#include "light-adjustment.h"

//////////////////////////////////////////////////
// Global variables
//////////////////////////////////////////////////
uint16_t phase_delay_time = PHASE_DELAY_DEFAULT;
bool update_alarm_val = true;

bool phase_delay_timer_on = false;
gptimer_handle_t phase_delay_timer = NULL;
gptimer_alarm_config_t phase_delay_timer_alarm_config = {
    .alarm_count = 0,
    .flags.auto_reload_on_alarm = false,
};

bool power_hardware;

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
    ESP_LOGI(TAG_subsystem1, "meas rate: %d", LTR303_REG_MEAS_RATE);

    // set measurement mode to active
    // ltr303_register_write_byte(LTR303_REG_CONTR, 1<<LTR303_REG_CONTR_MODE_BIT | 1<<LTR303_REG_CONTR_2XGAIN_BIT);
    ltr303_register_write_byte(LTR303_REG_CONTR, 1<<LTR303_REG_CONTR_MODE_BIT);
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
        phase_delay_timer_alarm_config.alarm_count = (device_power && power_hardware && (phase_delay_time != 8300)) ? phase_delay_time : 30000; // if device power off make phase delay timer > 8.3 ms
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

TickType_t lastTickTime;
//////////////////////////////////////////////////
// Power button (digital input)
//////////////////////////////////////////////////
static void IRAM_ATTR power_button_isr_handler(void* arg)
{
    bool reading = gpio_get_level(GPIO_INPUT_POWER_BUTTON_PIN);
    if (abs(xTaskGetTickCountFromISR() - lastTickTime) < 100) {
        power_hardware = false;
    }
    else {
        power_hardware = reading;
    }
    lastTickTime = xTaskGetTickCountFromISR();
    update_alarm_val = true;
}

void power_button_setup(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = POWER_BUTTON_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_POWER_BUTTON_PIN, power_button_isr_handler, NULL);
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
// Update light level
//////////////////////////////////////////////////
static uint16_t read_light_level() {
    uint8_t visible_and_infrared[2];
    ltr303_register_read(LTR303_REG_DATA_CH1_0, visible_and_infrared, 2); // ch 1 - visible and infrared
    uint8_t infrared[2];
    ltr303_register_read(LTR303_REG_DATA_CH0_0, infrared, 2); // ch 0 - infrared

    uint16_t light_meas_lux = visible_and_infrared[1]<<8 | visible_and_infrared[0];

    return light_meas_lux;
}

//////////////////////////////////////////////////
// Light range calibration
//////////////////////////////////////////////////
static uint16_t range_calibration() {
    uint16_t temp = phase_delay_time;
    uint8_t delay_seconds = 5;

   // Determine brightest value
   phase_delay_time = 50;
   update_alarm_val = true;
   vTaskDelay(delay_seconds * 100);
   uint16_t max_brightness = read_light_level();
   // Determine dimmest value
   phase_delay_time = 8300;
   update_alarm_val = true;
   vTaskDelay(delay_seconds * 100);
   uint16_t min_brightness = read_light_level();

   // Set range
   uint16_t lux_range = (max_brightness - min_brightness);
   ESP_LOGI(TAG_subsystem1, "lux range: %d lux", lux_range);
   phase_delay_time = temp; // return phase delay time to previous value
   return lux_range;
}

//////////////////////////////////////////////////
// Update light level
//////////////////////////////////////////////////
static uint8_t update_light_level(uint16_t target_val_lux, uint16_t light_meas_lux, uint16_t tolerance_lux, uint8_t delay_seconds, uint16_t lux_range) {
    if (abs(target_val_lux - light_meas_lux) > 2*tolerance_lux) {
        phase_delay_time -= 0.8 * (target_val_lux - light_meas_lux)/lux_range * 8333;
        delay_seconds = 3;
    }
    else if (abs(target_val_lux - light_meas_lux) > tolerance_lux) {
        if (target_val_lux > light_meas_lux) {
            phase_delay_time -= 10;
        }
        else {
            phase_delay_time += 10;
        }
        delay_seconds = 15;
    }
    
    if (phase_delay_time > 50000) { // if phase_delay_time decreased and overflowed to very high value, reset to minimum time
        phase_delay_time = 100;
    }
    else if (phase_delay_time > 8500) { // if phase_delay_time increased above max, set to 8300 ms (turned to max value)
        phase_delay_time = 8300;
    }

    return delay_seconds;
}

//////////////////////////////////////////////////
// Compare target and measured light level
//////////////////////////////////////////////////
static uint8_t compare_light_level(uint16_t lux_range, uint8_t delay_seconds) {
    uint16_t target_val_lux = req_light_level * lux_range / 100; // convert user-set light level percentage to target value in lux
    uint16_t light_meas_lux = read_light_level();
    uint16_t tolerance_lux = 10;

    ESP_LOGI(TAG_subsystem1, "target: %d lux", target_val_lux);
    ESP_LOGI(TAG_subsystem1, "light measurement: %d lux", light_meas_lux);
    
    if (abs(target_val_lux - light_meas_lux) > tolerance_lux) { // if alarm val will be updated, set flag to true
        delay_seconds = update_light_level(target_val_lux, light_meas_lux, tolerance_lux, delay_seconds, lux_range);
        update_alarm_val = true;
    }

    ESP_LOGI(TAG_subsystem1, "phase delay time: %d", phase_delay_time);

    return delay_seconds;
}

//////////////////////////////////////////////////
//
//////////////////////////////////////////////////
void app_main() {
    // Subsystem 4 setup
    /* NVS(Non-Volatile Storage) is a partition in flash memory which stores key-value pairs.
    We can use NVS to store WiFi configuration.*/
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG_subsystem4, "ESP_WIFI_MODE_STA");
    WiFi_InitIn_StationMode();

    espnow_init();

    #if IS_MASTER
        Setup_HTTP_server();
    #endif

    // Subsystem 1 setup
    // lastTickTime = xTaskGetTickCount;
    // power_button_setup();
    // power_hardware = gpio_get_level(GPIO_INPUT_POWER_BUTTON_PIN);
    power_hardware = true;
    trigger_setup();

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG_subsystem1, "I2C initialized successfully");
    ltr303_setup();

    phase_delay_timer_setup();

    zero_crossing_setup();

    uint8_t delay_seconds = 3;

    // uint16_t lux_range = range_calibration();
    uint16_t lux_range = 150;

    while(1) {
        vTaskDelay(delay_seconds * 100);

        if (!device_power || !power_hardware) {  // if device set off just run through loop again
            update_alarm_val = true;
            delay_seconds = 3;
            continue;
        }

        delay_seconds = compare_light_level(lux_range, delay_seconds);
    }
}


// GPIO example:                        https://github.com/espressif/esp-idf/blob/v5.1/examples/peripherals/gpio/generic_gpio/
// GPIO documentation:                  https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/peripherals/gpio.html

// I2C example:                         https://github.com/espressif/esp-idf/blob/v5.1/examples/peripherals/i2c/i2c_simple/
// I2C documentation:                   https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/peripherals/i2c.html

// General purpose timer example:       https://github.com/espressif/esp-idf/tree/v5.1/examples/peripherals/timer_group/gptimer
// General purpose timer documentation: https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/peripherals/timer.html#_CPPv421timer_set_alarm_value13timer_group_t11timer_idx_t8uint64_t
