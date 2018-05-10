EXEC = spacedata

OBJECTS = \
	main.o \
	temp_sensor.o \
	magnetometer.o \
	lsm.o \

DEPS += \
	libchain \
	libmspware \
	libharvest \
	libmspuartlink \
	libedb \
	libio \
	libmsp \

override CFLAGS += \
	-Wno-unused-function \

ENABLE_GYRO = 0

CONFIG_EDB = 1

MAIN_CLOCK_FREQ = 1000000

export CLOCK_FREQ_ACLK = 32768
export CLOCK_FREQ_MCLK = $(MAIN_CLOCK_FREQ)
export CLOCK_FREQ_SMCLK = $(MAIN_CLOCK_FREQ)

export LIBMSP_CLOCK_SOURCE = DCO
export LIBMSP_DCO_FREQ = $(MAIN_CLOCK_FREQ)
export LIBMSP_SLEEP_TIMER = A.0.0
export LIBMSP_SLEEP_TIMER_CLK = ACLK
export LIBMSP_SLEEP_TIMER_DIV = 8*8

export LIBIO_BACKEND = hwuart
export LIBMSP_UART_IDX = 1
export LIBMSP_UART_PIN_TX = 2.5
export LIBMSP_UART_BAUDRATE = 115200
export LIBMSP_UART_CLOCK = SMCLK

#export LIBIO_BACKEND = swuart
#export LIBMSPSOFTUART_PORT_TX = 2.6
#export LIBMSPSOFTUART_BAUDRATE = 115200
#export LIBMSPSOFTUART_CLOCK_FREQ = $(MAIN_CLOCK_FREQ)
#export LIBMSPSOFTUART_TIMER = B.0.1

export LIBHARVEST_COMP_CHAN = 11
export LIBHARVEST_COMP_PORT = 2.4
export LIBHARVEST_COMP_REF = 23 # 2.0v * (3.3/(3.3+4.22)) / 1.2v * 32

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
else ifeq ($(BOARD),msp-exp430fr5994) # dummy clause: board version meaningless for LP
export LIBEDB_BOARD_VERSION_MAJOR = 1
export LIBEDB_BOARD_VERSION_MINOR = 2
else
$(error BOARD incosistent with libedb board setting)
endif

include ../Makefile.options
