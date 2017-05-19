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
	libharvest \

override CFLAGS += \
	-Wno-unused-function \

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

export LIBHARVEST_COMP_CHAN = 10
export LIBHARVEST_COMP_PORT = 2.3
#export LIBHARVEST_COMP_REF = 28 # 2.5v * (3.9/(3.9+5.23)) / 1.2v * 32
export LIBHARVEST_COMP_REF = 25 # 2.2v * (3.9/(3.9+5.23)) / 1.2v * 32

include $(LIB_ROOT)/libedb/bld/Makefile.target

include $(MAKER_ROOT)/Makefile.console

include ../Makefile.options
