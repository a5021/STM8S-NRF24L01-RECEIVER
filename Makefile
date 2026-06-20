CC      = sdcc
CFLAGS  = -mstm8 --out-fmt-ihx -Iinc --all-callee-saves
LDFLAGS = -mstm8 --out-fmt-ihx --iram-size 1024 --xram-size 0 --code-size 16384

SRC_DIR = src
BUILD_DIR = build
TARGET  = $(BUILD_DIR)/firmware

C_SOURCES = $(wildcard $(SRC_DIR)/*.c)
C_PATCHED = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=_patched.c)))

.PHONY: all preprocess clean

all: $(TARGET).hex

preprocess: $(C_PATCHED)

$(BUILD_DIR)/%_patched.c: $(SRC_DIR)/%.c inc/iostm8s003f3.h | $(BUILD_DIR)
	python3 iar2sdcc.py $< $@

$(TARGET).ihx: $(C_PATCHED)
	$(CC) $(CFLAGS) $(C_PATCHED) -o $@

$(TARGET).hex: $(TARGET).ihx
	$(CC) $(LDFLAGS) $(C_PATCHED) -o $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR) *.asm *.lst *.map *.lk *.sym *.adb *.cdb *.ihx