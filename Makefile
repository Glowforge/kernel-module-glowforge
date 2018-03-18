MODULE_NAME = glowforge
OUT = $(MODULE_NAME).ko
SRC_DIR = ./src

KERNEL_SRC ?= /usr/src/kernel

SRC = ledtrig_smooth.c io.c \
      pic.c pic_leds.c pic_api.c thermal.c \
      cnc_buffer.c cnc.c cnc_api.c cnc_pins.c \
      glowforge.c

ccflags-y += -I$(PWD) -Wno-unknown-pragmas -g -fno-omit-frame-pointer

obj-m += $(MODULE_NAME).o
$(MODULE_NAME)-objs := $(foreach srcfile,$(SRC),$(SRC_DIR)/$(srcfile:.c=.o))

.PHONY: all clean modules_install

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean

# run menu-based config tool
config:
	$(MAKE) -$(MAKEFLAGS) -C $(KERNEL_SRC) nconfig
	cp $(KERNEL_SRC)/.config $(PWD)/config.new

# pass any other command to the kernel Makefile directly
%:
	$(MAKE) -$(MAKEFLAGS) -C $(KERNEL_SRC) $@
