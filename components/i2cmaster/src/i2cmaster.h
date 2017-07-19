/**
 * @file i2cmaster.h
 * @brief header file of esp-wroom-32 I2C Master
 * @author m2enu
 * @date 2017.07.19
 */
#include <stdint.h>
#include <stdio.h>
#include "driver/i2c.h"

#define I2C_MASTER_ACK          0
#define I2C_MASTER_NACK         1

/** <!-- i2c_code_t {{{1 -->
 * @brief I2C communication return code
 */
typedef enum {
    I2C_CODE_OK = 0,    /*!< I2C communication SUCCESS */
    I2C_CODE_NG,        /*!< I2C communication FAILED */
    I2C_CODE_MAX,
} i2c_code_t;

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
                     uint32_t i2c_freq);

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
              uint16_t cnt);

/** <!-- i2c_rd {{{1 -->
 * @brief I2C master read function
 * @param dev_addr device address (w/o WR/RD flag)
 * @param reg_addr register address
 * @param *reg_data register write data
 * @param cnt write byte count
 * @param 0:OK, others:NG
 */
int8_t i2c_rd(uint8_t dev_addr,
              uint8_t reg_addr,
              uint8_t *reg_data,
              uint16_t cnt);

// end of file {{{1
// vim:ft=cpp:et:nowrap:fdm=marker
