MODULE_NAME = glowforge
OUT = $(MODULE_NAME).ko
SRC_DIR = ./src
ASM_DIR = ./asm
TOOLS_DIR = ./tools
SDMA_ASSEMBLER = PERL5LIB=$(TOOLS_DIR) $(TOOLS_DIR)/sdma_asm.pl

KERNEL_SRC ?= /usr/src/kernel

SRC = ledtrig_smooth.c io.c \
      pic.c pic_leds.c pic_api.c thermal.c \
      cnc_buffer.c cnc.c cnc_api.c cnc_pins.c \
      glowforge.c
ASM = sdma.asm

ccflags-y += -I$(PWD) -Wno-unknown-pragmas -g -fno-omit-frame-pointer

obj-m += $(MODULE_NAME).o
$(MODULE_NAME)-objs := $(foreach srcfile,$(SRC),$(SRC_DIR)/$(srcfile:.c=.o))

SDMA_SCRIPT = $(ASM)
SDMA_SCRIPT_ASSEMBLED = $(addprefix $(SRC_DIR)/,$(ASM:.asm=.asm.h))
vpath %.asm $(ASM_DIR)

.PHONY: all clean modules_install

all: $(SDMA_SCRIPT_ASSEMBLED)
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

$(SRC_DIR)/%.asm.h: %.asm
	$(SDMA_ASSEMBLER) $< > $@

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean
	rm -f $(SDMA_SCRIPT_ASSEMBLED)

# run menu-based config tool
config:
	$(MAKE) -$(MAKEFLAGS) -C $(KERNEL_SRC) nconfig
	cp $(KERNEL_SRC)/.config $(PWD)/config.new

# pass any other command to the kernel Makefile directly
%:
	$(MAKE) -$(MAKEFLAGS) -C $(KERNEL_SRC) $@
