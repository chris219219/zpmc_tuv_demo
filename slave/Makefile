CC = ../arm-toolchain/bin/arm-none-eabi-gcc
OBJCOPY = ../arm-toolchain/bin/arm-none-eabi-objcopy
RELEASE = 0

TARGET = slave

INCLUDES = \
	$(foreach dir, $(shell find . -type d), -I$(dir)) \
	$(foreach dir, $(shell find drivers -type d), -I$(dir))
CSOURCES = $(shell find . -name "*.c")
SSOURCES = $(shell find . -name "*.S")
BUILD_DIR = build
OBJDIR = $(BUILD_DIR)/obj
TARGET_ELF = $(BUILD_DIR)/$(TARGET).elf
TARGET_BIN = $(BUILD_DIR)/$(TARGET).bin

FPUFLAGS = \
	-mfloat-abi=hard \
	-mfpu=fpv5-sp-d16

CFLAGS = \
	-Wall \
	$(FPUFLAGS) \
	-mcpu=cortex-m7 \
	-mthumb \
	-DCPU_MIMX8ML8DVNLZ \
	-DCPU_MIMX8ML8DVNLZ_cm7 \
	-DDEBUG \
	-DSDK_DEBUGCONSOLE=1 \
	-DSERIAL_PORT_TYPE_UART=1

ifeq ($(RELEASE),1)
	CFLAGS += -Os
else
	CFLAGS += -O0 -g
endif

SFLAGS = \
	$(FPUFLAGS) \
    -D__STARTUP_CLEAR_BSS \
    -D__STARTUP_INITIALIZE_NONCACHEDATA \
    -mcpu=cortex-m7 \
    -mthumb

ifneq ($(RELEASE),1)
	SFLAGS += -DDEBUG
endif

LDFLAGS = \
    -mcpu=cortex-m7 \
    -Wall \
    -Wl,--print-memory-usage \
    -fno-common \
    -ffunction-sections \
    -fdata-sections \
    -ffreestanding \
    -fno-builtin \
    -mthumb \
    -Wl,--gc-sections \
    -Wl,-Map=output.map \
	$(FPUFLAGS) \
	--specs=nosys.specs \
	-T../linker-ram.ld

ifneq ($(RELEASE),1)
	LDFLAGS += -g
endif

.PHONY: debug release tests clean

debug: $(BUILD_DIR) $(TARGET_BIN)
release:
	$(MAKE) RELEASE=1

# list all object files to be linked
OBJS = \
	$(patsubst %.c, $(OBJDIR)/%.o, $(CSOURCES)) \
    $(patsubst %.S, $(OBJDIR)/%.o, $(SSOURCES))

$(BUILD_DIR):
	@mkdir -p $@

# compile C sources
$(OBJDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<

# compile assembly sources
$(OBJDIR)/%.o: %.S
	@mkdir -p $(dir $@)
	$(CC) -c $(SFLAGS) $(INCLUDES) -o $@ $<

# link object files to create the ELF file
$(TARGET_ELF): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

# create BIN file from ELF file
$(TARGET_BIN): $(TARGET_ELF)
	$(OBJCOPY) -O binary $< $@

clean:
	rm -rf $(BUILD_DIR)
	rm -f output.map
