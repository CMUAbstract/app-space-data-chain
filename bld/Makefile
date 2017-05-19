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

export LIBMSPSOFTUART_PORT_TX = 2.6
export LIBMSPSOFTUART_BAUDRATE = 115200
export LIBMSPSOFTUART_CLOCK_FREQ = 8000000
export LIBMSPSOFTUART_TIMER = B.0.1

export LIBHARVEST_COMP_CHAN = 10
export LIBHARVEST_COMP_PORT = 2.3
#export LIBHARVEST_COMP_REF = 28 # 2.5v * (3.9/(3.9+5.23)) / 1.2v * 32
export LIBHARVEST_COMP_REF = 25 # 2.2v * (3.9/(3.9+5.23)) / 1.2v * 32

# This is ugly. Ideally, libedb would stop being board aware>
# This would involve switches for individual workarounds, instead
# of conditioning the workarounds based on board setting.
ifeq ($(BOARD),sprite-app-v1.2)
export LIBEDB_BOARD_VERSION_MAJOR = 1
export LIBEDB_BOARD_VERSION_MINOR = 2
else
$(error BOARD incosistent with libedb board setting)
endif

include $(LIB_ROOT)/libedb/bld/Makefile.target

include $(MAKER_ROOT)/Makefile.console

include ../Makefile.options
