/**
 * @file rpi-gpio.h
 *
 * @ingroup raspberrypi_gpio
 *
 * @brief Raspberry Pi specific GPIO definitions.
 */

/*
 *  Copyright (c) 2015 Andre Marques <andre.lousa.marques at gmail.com>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_ARM_RASPBERRYPI_RPI_GPIO_H
#define LIBBSP_ARM_RASPBERRYPI_RPI_GPIO_H

#include <rtems.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief  Total number of GPIOs on the Raspberry Pi,
 *         including physical GPIO pins (available on the board hardware)
 *         and system GPIOs (only accessible to the system).
 */
#define GPIO_COUNT 54

#define RPI_DIGITAL_IN  7
#define RPI_DIGITAL_OUT 1
#define RPI_ALT_FUNC_0  4
#define RPI_ALT_FUNC_1  5
#define RPI_ALT_FUNC_2  6
#define RPI_ALT_FUNC_3  7
#define RPI_ALT_FUNC_4  3
#define RPI_ALT_FUNC_5  2

extern rtems_status_code gpio_select_jtag(void);
extern rtems_status_code gpio_select_spi_p1(void);
extern rtems_status_code gpio_select_i2c_p1_rev2(void);
  
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_RASPBERRYPI_RPI_GPIO_H */
