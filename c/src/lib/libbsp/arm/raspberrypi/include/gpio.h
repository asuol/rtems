/**
 * @file
 *
 * @ingroup raspberrypi_gpio
 *
 * @brief Raspberry Pi specific GPIO information.
 */

/*
 * Copyright (c) 2014 Andre Marques.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_ARM_RASPBERRYPI_GPIO_H
#define LIBBSP_ARM_RASPBERRYPI_GPIO_H

#include <rtems/libgpio.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define GPIO_PIN_COUNT 54

#define JTAG_PIN_COUNT 5

rtems_gpio_configuration JTAG_CONFIG[] = {
  { 4, ALT_FUNC_5 },  /* setup gpio 4 alt5 ARM_TDI */
  { 22, ALT_FUNC_4 }, /* setup gpio 22 alt4 ARM_TRST */
  { 24, ALT_FUNC_4 }, /* setup gpio 24 alt4 ARM_TDO */
  { 25, ALT_FUNC_4 }, /* setup gpio 25 alt4 ARM_TCK */
  { 27, ALT_FUNC_4 }  /* setup gpio 27 alt4 ARM_TMS */
};

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_RASPBERRYPI_GPIO_H */
