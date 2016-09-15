EXEC = spacedata

OBJECTS = \
	main.o \
	temp_sensor.o \
	magnetometer.o \
	gyro.o \

DEPS += \
	libmsp \
	libwispbase \
	libchain \
	libmspware \

override CFLAGS += \
	-Wno-unused-function \

BOARD = mspts430
CONFIG_EDB = 0
CONFIG_PRINTF_LIB = libmspsoftuart
CONFIG_LIBEDB_PRINTF = eif

export LIBMSPSOFTUART_PORT_TX = 3.5
export LIBMSPSOFTUART_BAUDRATE = 115200
export LIBMSPSOFTUART_CLOCK_FREQ = 8000000
export LIBMSPSOFTUART_TIMER = B.0.4

#include $(MAKER_ROOT)/Makefile.edb

include $(MAKER_ROOT)/Makefile.console
