#ifndef LIBBSP_ARM_RASPBERRYPI_GPIO_H
#define LIBBSP_ARM_RASPBERRYPI_GPIO_H

#include <rtems/libgpio.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

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
