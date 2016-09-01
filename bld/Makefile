EXEC = spacedata

OBJECTS = \
	main.o \
	temp_sensor.o \

DEPS += \
	libmsp \
	libwispbase \
	libchain \

BOARD = mspts430
CONFIG_EDB = 0
CONFIG_PRINTF_LIB = libmspconsole
CONFIG_LIBEDB_PRINTF = eif

#include $(MAKER_ROOT)/Makefile.gcc
#include $(MAKER_ROOT)/Makefile.board
#
#include $(MAKER_ROOT)/Makefile.edb
#include $(MAKER_ROOT)/Makefile.chain

include $(MAKER_ROOT)/Makefile.console
