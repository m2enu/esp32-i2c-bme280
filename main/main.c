/**
 * Copyright (C) 2017 m2enu
 *
 * @file main/main.c
 * @brief Bosch Sensortec BME280 Pressure sensor logger via I2C
 * @author m2enu
 * @date 2017/07/20
 */
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
static const char *TAG = "main"; //!< ESP_LOGx tag

// defines {{{1
#define I2C_NUM                     CONFIG_I2C_PORT_NUM //!< I2C port number
#define I2C_SDA                     CONFIG_I2C_PORT_SDA //!< GPIO num of I2C_SDA
#define I2C_SCL                     CONFIG_I2C_PORT_SCL //!< GPIO num of I2C_SCL
#define I2C_FREQ                    CONFIG_I2C_FREQ //!< I2C frequency
#define DEBUG_LED_BLINK             CONFIG_DEBUG_LED_BLINK //!< LED blink debug flag
#define GPIO_LED                    CONFIG_GPIO_LED //!< GPIO num of LED

#define BME280_I2C_ADDR             CONFIG_BME280_I2C_ADDR //!< BME280 device address
#define BME280_OVERSAMPLING_P       CONFIG_BME280_OSR_P //!< Oversampling rate of Pressure
#define BME280_OVERSAMPLING_T       CONFIG_BME280_OSR_T //!< Oversampling rate of Temperature
#define BME280_OVERSAMPLING_H       CONFIG_BME280_OSR_H //!< Oversampling rate of Humidity
#define BME280_WAIT_FORCED          CONFIG_BME280_WAIT_FORCED //!< wait time after oneshot

// function declarations {{{1
void delay_msec(uint32_t msec);
bool BME280_device_init(struct bme280_dev *dev);
void BME280_show_calib_data(struct bme280_calib_data *clb);
void BME280_show_sensor_data(struct bme280_data *comp_data);
bool BME280_oneshot(struct bme280_dev *dev, struct bme280_data *comp_data);
static void BME280_log(void *args);

// global members {{{1
struct bme280_dev m_dev; //!< BME280 device pointer
struct bme280_data m_comp_data; //!< BME280 compensated data pointer

/** <!-- delay_msec {{{1 -->
 * @brief delay function
 * @param[in] msec wait time in milliseconds
 * @return nothing
 */
void delay_msec(uint32_t msec)
{
    vTaskDelay(msec / portTICK_PERIOD_MS);
}

/** <!-- BME280_device_init {{{1 -->
 * @brief initialization of BME280 device
 * @param[in] dev BME280 device pointer
 * @return Result of BME280 device initialization
 * @retval false: OK
 * @retval true: NG
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
 * @param[in] clb BME280 calibration parameter pointer
 * @return nothing
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
 * @param[in] comp_data BME280 compensated data pointer
 * @return nothing
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
 * @param[in] dev BME280 device pointer
 * @param[out] comp_data BME280 compensated data pointer
 * @return Result of operation in forced mode
 * @retval false: OK
 * @retval true: NG
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
 * @return nothing
 */
static void BME280_log(void *args)
{
#if DEBUG_LED_BLINK
    int level = 0;
#endif
    while (true) {
        // get BME280 sensor data by forced mode
        BME280_oneshot(&m_dev, &m_comp_data);
        BME280_show_sensor_data(&m_comp_data);

        // blink LED
#if DEBUG_LED_BLINK
        gpio_set_level(GPIO_LED, level);
        level = !level;
#endif

        delay_msec(300);
    }
}

/** <!-- app_main {{{1 -->
 * @brief main function
 * @return nothing
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
