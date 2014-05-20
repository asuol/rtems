
rtems_tests_PROGRAMS = GPIO_pierre
GPIO_pierre_SOURCES = init.c rpi_gpio.c rpi_gpio.h

include $(RTEMS_ROOT)/make/custom/@RTEMS_BSP@.cfg
include $(top_srcdir)/../automake/compile.am
include $(top_srcdir)/../automake/leaf.am


AM_CPPFLAGS += -I$(top_srcdir)/include
AM_CPPFLAGS += -I$(top_srcdir)/../support/include

LINK_OBJS = $(GPIO_pierre_OBJECTS)
LINK_LIBS = $(GPIO_pierre_LDLIBS)

GPIO_pierre$(EXEEXT): $(GPIO_pierre_OBJECTS) $(GPIO_pierre_DEPENDENCIES)
	@rm -f GPIO_pierre$(EXEEXT)
	$(make-exe)

include $(top_srcdir)/../automake/local.am
