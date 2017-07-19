#include <stdint.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "bme280.h"

/* <!-- app_main {{{1 -->
 * @brief main function
 */
void app_main(void)
{
    struct bme280_dev dev = {
        .id = BME280_I2C_ADDR_SEC,
        .interface = BME280_I2C_INTF
    };
    int8_t ret = bme280_init(&dev);

    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    int level = 0;
    while (true) {
        gpio_set_level(GPIO_NUM_4, level);
        level = !level;
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

// end of file {{{1
// vim:ft=c:et:nowrap:fdm=marker
