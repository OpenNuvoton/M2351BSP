#
# Copyright (C) 2015-2017 Alibaba Group Holding Limited
#

CROSS_COMPILE 	?= arm-none-eabi-

CC              := $(CROSS_COMPILE)gcc
LD              := $(CROSS_COMPILE)ld

PWD             := $(shell pwd)
TW_DIR          = $(PWD)/../../../src/tw
XOR_INC         = $(PWD)/../inc
COMMON_INC      = $(TW_DIR)/../common/inc
TEE_INC         = $(TW_DIR)/inc

include $(TW_DIR)/defconfig

CFLAGS          := -I. -I$(XOR_INC) -I$(TEE_INC)
CFLAGS          += -mthumb -mcpu=cortex-m23 -mcmse -mthumb-interwork -Werror \
					-O2 -Werror-implicit-function-declaration \
					-Wstrict-prototypes -Wwrite-strings -fno-builtin \

include $(TW_DIR)/tee.cfg

SRCS := $(wildcard *.c *.cxx)
OBJS := $(patsubst %.cxx,%.o,$(patsubst %.c,%.o,$(SRCS)))

built-in.o: $(OBJS)
	$(LD) -r $^ -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $*.o

clean:
	rm -f *.o built-in.*
