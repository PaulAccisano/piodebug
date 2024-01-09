# Location of top-level MicroPython directory
MPY_DIR = ../Micropython

# Name of module
MOD = piodebug

# Source files (.c or .py)
SRC = $(wildcard *.c) $(wildcard *.py)

# Architecture to build for (x86, x64, armv6m, armv7m, xtensa, xtensawin)
ARCH = armv6m

# Include to get the rules for compiling and linking the module
include $(MPY_DIR)/py/dynruntime.mk