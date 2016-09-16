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

override BOARD = sprite-app

ENABLE_GYRO = 0

CONFIG_EDB = 1
CONFIG_PRINTF_LIB = libmspsoftuart
CONFIG_LIBEDB_PRINTF = eif

export LIBMSP_CLOCK_SOURCE = DCO
export LIBMSP_DCO_FREQ = 8000000ull

export LIBMSPSOFTUART_PORT_TX = 3.5
export LIBMSPSOFTUART_BAUDRATE = 115200
export LIBMSPSOFTUART_CLOCK_FREQ = 8000000
export LIBMSPSOFTUART_TIMER = B.0.4

include $(LIB_ROOT)/libedb/bld/Makefile.target

include $(MAKER_ROOT)/Makefile.console

include ../Makefile.options
