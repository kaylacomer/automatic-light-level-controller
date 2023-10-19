#ifndef LIGHT_ADJUSTMENT_HEADER_H
#define LIGHT_ADJUSTMENT_HEADER_H

#define PHASE_DELAY_DEFAULT 8000

//////////////////////////////////////////////////
// System setup
//////////////////////////////////////////////////
#define CONFIG_FREERTOS_HZ 1000
#define TICK_WAIT_PERIOD 100
#define CONFIG_COMPILER_OPTIMIZATION -O2

static const char *TAG_subsystem1 = "light-adjustment";

#define ESP_INTR_FLAG_DEFAULT 0

#define ON  1
#define OFF 0

//////////////////////////////////////////////////
// Pin mapping
//////////////////////////////////////////////////
#define GPIO_INPUT_ZERO_CROSS_PIN     2
#define ZERO_CROSSING_PIN_SEL  (1ULL<<GPIO_INPUT_ZERO_CROSS_PIN)

#define GPIO_INPUT_LTR303_INTR_PIN     5
#define LTR303_INTR_PIN_SEL (1ULL<<GPIO_INPUT_LTR303_INTR_PIN)

#define GPIO_OUTPUT_TRIAC_TRIGGER_PIN    18
#define GPIO_OUTPUT_TRIAC_TRIGGER_PIN_SEL  (1ULL<<GPIO_OUTPUT_TRIAC_TRIGGER_PIN)

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

#endif