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

# TODO: package mspware as a normal dep
CFLAGS += -I$(LIB_ROOT)
LFLAGS += -L$(LIB_ROOT)/mspware
LIBS += -lmspware

override CFLAGS += \
	-Wno-unused-function \

BOARD = mspts430
CONFIG_EDB = 0
CONFIG_PRINTF_LIB = libmspconsole
CONFIG_LIBEDB_PRINTF = eif

#include $(MAKER_ROOT)/Makefile.board
#
#include $(MAKER_ROOT)/Makefile.edb
#include $(MAKER_ROOT)/Makefile.chain

include $(MAKER_ROOT)/Makefile.console
