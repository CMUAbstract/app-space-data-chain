EXEC = spacedata.out

OBJECTS = \
	main.o \
	temp_sensor.o \

CONFIG_EDB = 0
CONFIG_PRINTF_LIB = libmspsoftuart
CONFIG_LIBEDB_PRINTF = eif

include $(MAKER_ROOT)/Makefile.gcc
include $(MAKER_ROOT)/Makefile.board
include $(MAKER_ROOT)/Makefile.console
include $(MAKER_ROOT)/Makefile.edb
include $(MAKER_ROOT)/Makefile.chain

CFLAGS += \
	-I$(WISP_BASE_ROOT)/Sensors \
	-I$(WISP_BASE_ROOT)/wired \
	-I$(LIBMSPBUILTINS_ROOT)/src/include \
	-I$(LIBMSPMATH_ROOT)/src/include \

LFLAGS += \
	-L$(LIBMSPMATH_ROOT)/bld/gcc \

LIBS += \
	-lmspmath \

VPATH = ../../src
