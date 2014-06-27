
rtems_tests_PROGRAMS = LIBGPIO_TEST_IQR
LIBGPIO_TEST_IQR_SOURCES = init.c

#dist_rtems_tests_DATA = LIBGPIO_TEST_IQR.scn
#dist_rtems_tests_DATA += LIBGPIO_TEST_IQR.doc

include $(RTEMS_ROOT)/make/custom/@RTEMS_BSP@.cfg
include $(top_srcdir)/../automake/compile.am
include $(top_srcdir)/../automake/leaf.am


LINK_OBJS = $(LIBGPIO_TEST_IQR_OBJECTS)
LINK_LIBS = $(LIBGPIO_TEST_IQR_LDLIBS)

LIBGPIO_TEST_IQR$(EXEEXT): $(LIBGPIO_TEST_IQR_OBJECTS) $(LIBGPIO_TEST_IQR_DEPENDENCIES)
	@rm -f LIBGPIO_TEST_IQR$(EXEEXT)
	$(make-exe)

include $(top_srcdir)/../automake/local.am
