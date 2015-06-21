/**
 * @file gpio-interfaces.c
 *
 * @ingroup raspberrypi_gpio
 *
 * @brief Raspberry PI GPIO interface definitions.
 */

/*
 *  Copyright (c) 2015 Andre Marques <andre.lousa.marques at gmail.com>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

gpio_specific_data alt_func_def[] = {
  {io_function: RPI_ALT_FUNC_0, pin_data: NULL},
  {io_function: RPI_ALT_FUNC_1, pin_data: NULL},
  {io_function: RPI_ALT_FUNC_2, pin_data: NULL},
  {io_function: RPI_ALT_FUNC_3, pin_data: NULL},
  {io_function: RPI_ALT_FUNC_4, pin_data: NULL},
  {io_function: RPI_ALT_FUNC_5, pin_data: NULL}
};

gpio_pin_conf arm_tdi = {
 pin_number: 4,
 function: BSP_SPECIFIC,
 pull_mode: NO_PULL_RESISTOR,
 interrupt: NULL,
 output_enabled: FALSE,
 logic_invert: FALSE,
 bsp_specific: &alt_func_def[5]
};

gpio_pin_conf arm_trst = {
 pin_number: 22,
 function: BSP_SPECIFIC,
 pull_mode: NO_PULL_RESISTOR,
 interrupt: NULL,
 output_enabled: FALSE,
 logic_invert: FALSE,
 bsp_specific: &alt_func_def[4]
};

gpio_pin_conf arm_tdo = {
 pin_number: 24,
 function: BSP_SPECIFIC,
 pull_mode: NO_PULL_RESISTOR,
 interrupt: NULL,
 output_enabled: FALSE,
 logic_invert: FALSE,
 bsp_specific: &alt_func_def[4]
};

gpio_pin_conf arm_tck = {
 pin_number: 25,
 function: BSP_SPECIFIC,
 pull_mode: NO_PULL_RESISTOR,
 interrupt: NULL,
 output_enabled: FALSE,
 logic_invert: FALSE,
 bsp_specific: &alt_func_def[4]
};

gpio_pin_conf arm_tms = {
 pin_number: 27,
 function: BSP_SPECIFIC,
 pull_mode: NO_PULL_RESISTOR,
 interrupt: NULL,
 output_enabled: FALSE,
 logic_invert: FALSE,
 bsp_specific: &alt_func_def[4]
};

gpio_pin_conf spi_p1_miso = {
 pin_number: 7,
 function: BSP_SPECIFIC,
 pull_mode: NO_PULL_RESISTOR,
 interrupt: NULL,
 output_enabled: FALSE,
 logic_invert: FALSE,
 bsp_specific: &alt_func_def[0]
};

gpio_pin_conf spi_p1_mosi = {
 pin_number: 8,
 function: BSP_SPECIFIC,
 pull_mode: NO_PULL_RESISTOR,
 interrupt: NULL,
 output_enabled: FALSE,
 logic_invert: FALSE,
 bsp_specific: &alt_func_def[0]
};

gpio_pin_conf spi_p1_sclk = {
 pin_number: 9,
 function: BSP_SPECIFIC,
 pull_mode: NO_PULL_RESISTOR,
 interrupt: NULL,
 output_enabled: FALSE,
 logic_invert: FALSE,
 bsp_specific: &alt_func_def[0]
};

gpio_pin_conf spi_p1_ce_0 = {
 pin_number: 10,
 function: BSP_SPECIFIC,
 pull_mode: NO_PULL_RESISTOR,
 interrupt: NULL,
 output_enabled: FALSE,
 logic_invert: FALSE,
 bsp_specific: &alt_func_def[0]
};

gpio_pin_conf spi_p1_ce_1 = {
 pin_number: 11,
 function: BSP_SPECIFIC,
 pull_mode: NO_PULL_RESISTOR,
 interrupt: NULL,
 output_enabled: FALSE,
 logic_invert: FALSE,
 bsp_specific: &alt_func_def[0]
};

gpio_pin_conf i2c_p1_rev2_sda = {
 pin_number: 2,
 function: BSP_SPECIFIC,
 pull_mode: PULL_UP,
 interrupt: NULL,
 output_enabled: FALSE,
 logic_invert: FALSE,
 bsp_specific: &alt_func_def[0]
};

gpio_pin_conf i2c_p1_rev2_scl = {
 pin_number: 3,
 function: BSP_SPECIFIC,
 pull_mode: PULL_UP,
 interrupt: NULL,
 output_enabled: FALSE,
 logic_invert: FALSE,
 bsp_specific: &alt_func_def[0]
};
