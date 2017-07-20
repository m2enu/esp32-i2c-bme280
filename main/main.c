#include <stdint.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "bme280.h"
#include "i2cmaster.h"

// ESP_LOG* TAG
static const char *TAG = "main";

// defines {{{1
// TODO: append parameters into Kconfig.projbuild
#define I2C_NUM                     I2C_NUM_0
#define I2C_SDA                     GPIO_NUM_19
#define I2C_SCL                     GPIO_NUM_18
#define I2C_FREQ                    100000
#define GPIO_LED                    GPIO_NUM_4
#define BME280_I2C_ADDR             BME280_I2C_ADDR_SEC
#define BME280_OVERSAMPLING_P       BME280_OVERSAMPLING_4X
#define BME280_OVERSAMPLING_T       BME280_OVERSAMPLING_4X
#define BME280_OVERSAMPLING_H       BME280_OVERSAMPLING_4X
#define BME280_WAIT_FORCED          100
#define DEBUG_LED_BLINK             1

// function declarations {{{1
void delay_msec(uint32_t msec);
bool BME280_device_init(struct bme280_dev *dev);
void BME280_show_calib_data(struct bme280_calib_data *clb);
void BME280_show_sensor_data(struct bme280_data *comp_data);
bool BME280_oneshot(struct bme280_dev *dev, struct bme280_data *comp_data);
static void BME280_log(void *args);

// global members {{{1
struct bme280_dev m_dev;
struct bme280_data m_comp_data;

/** <!-- delay_msec {{{1 -->
 * @brief delay function
 * @param msec wait time [msec]
 */
void delay_msec(uint32_t msec)
{
    vTaskDelay(msec / portTICK_PERIOD_MS);
}

/** <!-- BME280_device_init {{{1 -->
 * @brief initialization of BME280 device
 * @param dev BME280 device pointer
 * @return OK:false, NG:true
 */
bool BME280_device_init(struct bme280_dev *dev)
{
    dev->id = BME280_I2C_ADDR;
    dev->interface = BME280_I2C_INTF;
    dev->read = i2c_rd;
    dev->write = i2c_wr;
    dev->delay_ms = delay_msec;

    int8_t ret = bme280_init(dev);
    ESP_LOGD(TAG, "bme280_init return code: %d", ret);
    ESP_LOGD(TAG, "BMP280 chip_id: 0x%02X", dev->chip_id);
    BME280_show_calib_data(&dev->calib_data);
    return (ret != BME280_OK);
}

/** <!-- BME280_show_calib_data {{{1 -->
 * @brief debug function: show BME280 calibration datas
 */
void BME280_show_calib_data(struct bme280_calib_data *clb)
{
    ESP_LOGD(TAG, "dig_T1=%8d", clb->dig_T1);
    ESP_LOGD(TAG, "dig_T2=%8d", clb->dig_T2);
    ESP_LOGD(TAG, "dig_T3=%8d", clb->dig_T3);
    ESP_LOGD(TAG, "dig_P1=%8d", clb->dig_P1);
    ESP_LOGD(TAG, "dig_P2=%8d", clb->dig_P2);
    ESP_LOGD(TAG, "dig_P3=%8d", clb->dig_P3);
    ESP_LOGD(TAG, "dig_P4=%8d", clb->dig_P4);
    ESP_LOGD(TAG, "dig_P5=%8d", clb->dig_P5);
    ESP_LOGD(TAG, "dig_P6=%8d", clb->dig_P6);
    ESP_LOGD(TAG, "dig_P7=%8d", clb->dig_P7);
    ESP_LOGD(TAG, "dig_P8=%8d", clb->dig_P8);
    ESP_LOGD(TAG, "dig_P9=%8d", clb->dig_P9);
    ESP_LOGD(TAG, "dig_H1=%8d", clb->dig_H1);
    ESP_LOGD(TAG, "dig_H2=%8d", clb->dig_H2);
    ESP_LOGD(TAG, "dig_H3=%8d", clb->dig_H3);
    ESP_LOGD(TAG, "dig_H4=%8d", clb->dig_H4);
    ESP_LOGD(TAG, "dig_H5=%8d", clb->dig_H5);
    ESP_LOGD(TAG, "dig_H6=%8d", clb->dig_H6);
}

/** <!-- BME280_show_sensor_data {{{1 -->
 * @brief debug function: show BME280 sensor datas
 * @param comp_data BME280 compensated data pointer
 */
void BME280_show_sensor_data(struct bme280_data *comp_data)
{
#ifdef FLOATING_POINT_REPRESENTATION
    ESP_LOGD(TAG, "%7.2fdegC  %7.2fPa  %7.2f%%",
             comp_data->temperature, comp_data->pressure, comp_data->humidity);
#else
    ESP_LOGD(TAG, "%ddegC  %dPa  %d%%",
             comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}

/** <!-- BME280_oneshot {{{1 -->
 * @brief get BME280 data in forced mode
 * @param dev BME280 device pointer
 * @param comp_data BME280 compensated data pointer
 * @return OK:false, NG:true
 */
bool BME280_oneshot(struct bme280_dev *dev,
                    struct bme280_data *comp_data)
{
    const int8_t retry = 5;
    int8_t n_try = retry;
    int8_t ret;
    uint8_t cfg;

    /* Continuously get the sensor data */
    while (n_try > 0) {
        dev->settings.osr_h = BME280_OVERSAMPLING_H;
        dev->settings.osr_p = BME280_OVERSAMPLING_P;
        dev->settings.osr_t = BME280_OVERSAMPLING_T;

        // put the device to forced mode
        cfg = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL;
        ret = bme280_set_sensor_settings(cfg, dev);
        ret = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
        dev->delay_ms(BME280_WAIT_FORCED);

        // read out sensor datas
        cfg = BME280_PRESS | BME280_HUM | BME280_TEMP;
        ret = bme280_get_sensor_data(cfg, comp_data, dev);
        if (ret == BME280_OK) {
            break;
        }
        ESP_LOGE(TAG, "Error at BME280_oneshot: %d (%d/%d)",
                 ret, (retry - n_try + 1), retry);
        n_try--;
    }
    return (ret != BME280_OK);
}

/** <!-- BME280_log {{{1 -->
 * @brief BME280 logger loop
 */
static void BME280_log(void *args)
{
    int level = 0;
    while (true) {
        // get BME280 sensor data by forced mode
        BME280_oneshot(&m_dev, &m_comp_data);
        BME280_show_sensor_data(&m_comp_data);

        // blink LED
#ifdef DEBUG_LED_BLINK
        gpio_set_level(GPIO_LED, level);
        level = !level;
#else
#endif

        delay_msec(300);
    }
}

/** <!-- app_main {{{1 -->
 * @brief main function
 */
void app_main(void)
{
    // initialize I2C master
    i2c_master_init(I2C_NUM, I2C_SDA, I2C_SCL, true, true, I2C_FREQ);
    // initialize BME280 device
    BME280_device_init(&m_dev);
    // initialize GPIO
    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);

    xTaskCreate(&BME280_log, "BME280_log", 2048, NULL, 5, NULL);
}

// end of file {{{1
// vim:ft=c:et:nowrap:fdm=marker
