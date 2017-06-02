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
	libmspuartlink \

override CFLAGS += \
	-Wno-unused-function \

ENABLE_GYRO = 0

CONFIG_EDB = 1
CONFIG_PRINTF_LIB = libmspconsole
CONFIG_LIBEDB_PRINTF = eif

MAIN_CLOCK_FREQ = 8000000

export CLOCK_FREQ_ACLK = 32768
export CLOCK_FREQ_SMCLK = $(MAIN_CLOCK_FREQ)

export LIBMSP_CLOCK_SOURCE = DCO
export LIBMSP_DCO_FREQ = $(MAIN_CLOCK_FREQ)

# Dev UART
export LIBMSP_UART_IDX = 0
export LIBMSP_UART_PIN_TX = 1.5
export LIBMSP_UART_BAUDRATE = 115200
export LIBMSP_UART_CLOCK = SMCLK

#export LIBMSPSOFTUART_PORT_TX = 2.6
#export LIBMSPSOFTUART_BAUDRATE = 115200
#export LIBMSPSOFTUART_CLOCK_FREQ = $(MAIN_CLOCK_FREQ)
#export LIBMSPSOFTUART_TIMER = B.0.1

export LIBHARVEST_COMP_CHAN = 10
export LIBHARVEST_COMP_PORT = 2.3
export LIBHARVEST_COMP_REF = 25 # 2.2v * (3.9/(3.9+5.23)) / 1.2v * 32

export LIBMSPUARTLINK_PIN_TX = 2.0
export LIBMSPUARTLINK_UART_IDX = 0
export LIBMSPUARTLINK_BAUDRATE = 4800
export LIBMSPUARTLINK_CLK = ACLK

export LIBWISPBASE_INCLUDE_UART = 0

export LIBEDB_ENABLE_UART = 0

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
