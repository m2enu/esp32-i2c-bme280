/**
 * @file i2cmaster.c
 * @brief esp-wroom-32 I2C Master
 * @author m2enu
 * @date 2017.07.19
 */
#include <stdint.h>
#include <stdio.h>
#include "esp_log.h"
#include "i2cmaster.h"

static const char *TAG = "I2C_M";
static i2c_port_t m_i2c_num = I2C_NUM_0;

/** <!-- i2c_master_init {{{1 -->
 * @brief initialize I2C Master
 * @param i2c_num I2C port number (0, 1)
 * @param sda_io_num GPIO port number of SDA
 * @param scl_io_num GPIO port number of SCL
 * @param sda_pullup_en true: pullup SDA
 * @param scl_pullup_en true: pullup SCL
 * @param i2c_freq I2C clock frequency [Hz]
 * @return false:OK, true:NG
 */
bool i2c_master_init(i2c_port_t i2c_num,
                     gpio_num_t sda_io_num,
                     gpio_num_t scl_io_num,
                     bool sda_pullup_en,
                     bool scl_pullup_en,
                     uint32_t i2c_freq
) {
    if ((i2c_num < 0) || (i2c_num > 1)) {
        ESP_LOGE(TAG, "invalid I2C port number: %d", i2c_num);
        return true;
    }
    m_i2c_num = i2c_num;

    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_io_num,
        .scl_io_num = scl_io_num,
        .sda_pullup_en = sda_pullup_en,
        .scl_pullup_en = scl_pullup_en,
        .master.clk_speed = i2c_freq,
    };
    esp_err_t err;
    err = i2c_param_config(m_i2c_num, &cfg);
    ESP_ERROR_CHECK(err);

    const size_t rxbuf = 0; // receiveing buffer size for slave mode
    const size_t txbuf = 0; // sending    buffer size for slave mode
    const int32_t flags = 0; // Flags used to allocate the interrupt
    err = i2c_driver_install(m_i2c_num, cfg.mode, rxbuf, txbuf, flags);
    ESP_ERROR_CHECK(err);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C master Initialization Failed: %d", err);
        return true;
    }
    return false;
}

/** <!-- i2c_wr {{{1 -->
 * @brief I2C master write function
 * @param dev_addr device address (w/o WR/RD flag)
 * @param reg_addr register address
 * @param *reg_data register write data
 * @param cnt write byte count
 * @param 0:OK, others:NG
 */
int8_t i2c_wr(uint8_t dev_addr,
              uint8_t reg_addr,
              uint8_t *reg_data,
              uint8_t cnt
) {
    uint8_t dev_adwr = (dev_addr << 1) | I2C_MASTER_WRITE;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_adwr, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, cnt, true);
    i2c_master_stop(cmd);

    esp_err_t ret;
    ret = i2c_master_cmd_begin(m_i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %d", ret);
        return I2C_CODE_NG;
    }
    return I2C_CODE_OK;
}

/** <!-- i2c_rd {{{1 -->
 * @brief I2C master read function
 * @param dev_addr device address (w/o WR/RD flag)
 * @param reg_addr register address
 * @param *reg_data register read data
 * @param cnt read byte count
 * @param 0:OK, others:NG
 */
int8_t i2c_rd(uint8_t dev_addr,
              uint8_t reg_addr,
              uint8_t *reg_data,
              uint8_t cnt
) {
    uint8_t dev_adwr = (dev_addr << 1) | I2C_MASTER_WRITE;
    uint8_t dev_adrd = (dev_addr << 1) | I2C_MASTER_READ;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_adwr, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_adrd, true);
    if (cnt > 1) {
        i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret;
    ret = i2c_master_cmd_begin(m_i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %d", ret);
        return I2C_CODE_NG;
    }
    return I2C_CODE_OK;
}

// end of file {{{1
// vim:ft=c:et:nowrap:fdm=marker
