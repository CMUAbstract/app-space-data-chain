export BOARD ?= sprite-app-v1.2
export VOLTAGE = 2400

TOOLCHAINS = \
	gcc \
	clang \
	mementos \
	dino \
	edbprof \

include ext/maker/Makefile

# Paths to toolchains here if not in or different from defaults in Makefile.env

TOOLCHAIN_ROOT = /opt/ti/mspgcc
