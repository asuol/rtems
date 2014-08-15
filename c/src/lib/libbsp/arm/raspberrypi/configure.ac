##
#
# @file
#
# @brief Configure script of LibBSP for the Raspberry Pi boards.
#

AC_PREREQ([2.68])
AC_INIT([rtems-c-src-lib-libbsp-arm-raspberrypi],[_RTEMS_VERSION],[http://www.rtems.org/bugzilla])
AC_CONFIG_SRCDIR([bsp_specs])
RTEMS_TOP(../../../../../..)

RTEMS_CANONICAL_TARGET_CPU
AM_INIT_AUTOMAKE([no-define nostdinc foreign 1.12.2])
RTEMS_BSP_CONFIGURE

RTEMS_PROG_CC_FOR_TARGET
RTEMS_CANONICALIZE_TOOLS
RTEMS_PROG_CCAS

RTEMS_CHECK_NETWORKING
AM_CONDITIONAL(HAS_NETWORKING,test "$HAS_NETWORKING" = "yes")

RTEMS_BSPOPTS_SET([BSP_START_RESET_VECTOR],[*],[])
RTEMS_BSPOPTS_HELP([BSP_START_RESET_VECTOR],[reset vector address for BSP start])

RTEMS_BSPOPTS_SET([I2C_IO_MODE],[*],[1])
RTEMS_BSPOPTS_HELP([I2C_IO_MODE],[Define to 1 to use interrupt-driven I/O with the Raspberry Pi I2C bus. If defined to other value the access will be polled-driven.])

RTEMS_BSPOPTS_SET([SPI_IO_MODE],[*],[1])
RTEMS_BSPOPTS_HELP([SPI_IO_MODE],[Define to 1 to use interrupt-driven I/O with the Raspberry Pi SPI bus. If defined to other value the access will be polled-driven.])

RTEMS_BSPOPTS_SET([BSP_ENABLE_SPI],[*],[1])
RTEMS_BSPOPTS_HELP([BSP_ENABLE_SPI],[Define to 1 to use the SPI bus. This will register the bus in /dev/spi and also the 23k256 SRAM SPI device in /dev/spi.23k256 .])

RTEMS_BSPOPTS_SET([BSP_ENABLE_I2C],[*],[1])
RTEMS_BSPOPTS_HELP([BSP_ENABLE_I2C],[Define to 1 to use the I2C bus. This will register the bus in /dev/i2c and also the mcp23008 gpio port expander I2C device in /dev/i2c.mcp23008 .])

RTEMS_BSP_CLEANUP_OPTIONS(0, 0)
RTEMS_BSP_LINKCMDS

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
