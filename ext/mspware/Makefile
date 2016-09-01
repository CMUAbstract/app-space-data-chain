# 
# Simple makefile to build inih as static library.
#

DEVICE  = msp430fr5969
SUPPORT_FILE_DIRECTORY = /opt/ti-mspgcc/include

SRC = $(wildcard *.c)
OBJ = $(SRC:.c=.o)
OUT = libmspware.a
INCLUDES = -I.
CCFLAGS = -I $(SUPPORT_FILE_DIRECTORY) -I. -mmcu=$(DEVICE) -O2 -g -fPIC \
	-g -fno-builtin-puts -fno-builtin-putc -fno-builtin-printf \

CCC = msp430-elf-gcc
LDFLAGS = -g

.SUFFIXES: .c

default: $(OUT)

.c.o:
	$(CCC) $(INCLUDES) $(CCFLAGS) $(EXTRACCFLAGS) -c $< -o $@

$(OUT): $(OBJ)
	ar rcs $(OUT) $(OBJ)

clean:
	rm -f $(OBJ) $(OUT)
