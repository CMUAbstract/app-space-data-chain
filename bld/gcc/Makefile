EXEC = spacedata.out

OBJECTS = \
	main.o \
	temp_sensor.o \
	magnetometer.o \
	gyro.o \

BOARD = mspts430
CONFIG_EDB = 0
CONFIG_PRINTF_LIB = libmspconsole
CONFIG_LIBEDB_PRINTF = eif

LIBMSPWARE = /home/vagrant/cvsandbox/chained-apps/space-data/src

include $(MAKER_ROOT)/Makefile.gcc
include $(MAKER_ROOT)/Makefile.board
include $(MAKER_ROOT)/Makefile.console
include $(MAKER_ROOT)/Makefile.edb
include $(MAKER_ROOT)/Makefile.chain

CFLAGS += \
	-Wno-unused-function \
	-I$(WISP_BASE_ROOT)/Sensors \
	-I$(WISP_BASE_ROOT)/wired \
	-I$(LIBMSPBUILTINS_ROOT)/src/include \
	-I$(LIBMSPMATH_ROOT)/src/include \
	-I$(LIBMSPPRINTF_ROOT)/src/include \

LFLAGS += \
	-L$(LIBMSPMATH_ROOT)/bld/gcc \
	-L$(LIBMSPPRINTF_ROOT)/bld/gcc \
	-L$(LIBMSPWARE)/mspware

LIBS += \
	-lwisp-base \
	-lmspmath \
	-lmspprintf \
	-lmspware \

VPATH = ../../src
