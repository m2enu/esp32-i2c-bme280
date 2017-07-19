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

/*!< @brief ESP_LOG* TAG */
static const char *TAG = "main";

/** <!-- defines {{{1 -->
 */
// TODO: append parameters into Kconfig.projbuild
#define I2C_NUM                     I2C_NUM_0
#define I2C_SDA                     GPIO_NUM_19
#define I2C_SCL                     GPIO_NUM_18
#define I2C_FREQ                    100000
#define BME280_I2C_ADDR             BME280_I2C_ADDR_SEC

/** <!-- delay_msec {{{1 -->
 * @brief delay function
 * @param msec wait time [msec]
 */
void delay_msec(uint32_t msec)
{
    vTaskDelay(msec / portTICK_PERIOD_MS);
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

/** <!-- app_main {{{1 -->
 * @brief main function
 */
void app_main(void)
{
    // initialize I2C master
    i2c_master_init(I2C_NUM, I2C_SDA, I2C_SCL, true, true, I2C_FREQ);

    // initialize BME280 device
    struct bme280_dev dev = {
        .id = BME280_I2C_ADDR,
        .interface = BME280_I2C_INTF,
        .read = i2c_rd,
        .write = i2c_wr,
        .delay_ms = delay_msec
    };
    int8_t ret = bme280_init(&dev);
    ESP_LOGD(TAG, "bme280_init return code: %d", ret);
    ESP_LOGD(TAG, "BMP280 chip_id: 0x%02X", dev.chip_id);
    BME280_show_calib_data(&dev.calib_data);

    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    int level = 0;
    while (true) {
        gpio_set_level(GPIO_NUM_4, level);
        level = !level;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// end of file {{{1
// vim:ft=c:et:nowrap:fdm=marker
