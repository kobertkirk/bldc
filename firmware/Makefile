# STM32G431CB firmware for the 48 V / 35 A hub motor controller
# Requires: arm-none-eabi-gcc + newlib (e.g. apt install gcc-arm-none-eabi libnewlib-arm-none-eabi)
# Flash:    make flash   (st-flash)   or   make flash-ocd   (openocd + ST-Link)

TARGET  = bldc48
PREFIX ?= arm-none-eabi-
CC      = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
SIZE    = $(PREFIX)size

SRC = $(wildcard src/*.c)
OBJ = $(patsubst src/%.c,build/%.o,$(SRC))

ARCH    = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS  = $(ARCH) -O2 -g3 -std=c11 -Wall -Wextra -Wshadow -Wdouble-promotion \
          -fsingle-precision-constant -fno-math-errno -ffunction-sections -fdata-sections \
          -DSTM32G431xx -Isrc -Icmsis/device -Icmsis/core
LDFLAGS = $(ARCH) -Tstm32g431cb.ld -Wl,--gc-sections -Wl,-Map=build/$(TARGET).map \
          --specs=nano.specs -Wl,--print-memory-usage
LDLIBS  = -lm

all: build/$(TARGET).elf build/$(TARGET).bin build/$(TARGET).hex

build/%.o: src/%.c src/*.h | build
	$(CC) $(CFLAGS) -c $< -o $@

build/$(TARGET).elf: $(OBJ) stm32g431cb.ld
	$(CC) $(LDFLAGS) $(OBJ) $(LDLIBS) -o $@
	$(SIZE) $@

build/$(TARGET).bin: build/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

build/$(TARGET).hex: build/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

build:
	mkdir -p build

flash: build/$(TARGET).bin
	st-flash --reset write $< 0x08000000

flash-ocd: build/$(TARGET).elf
	openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "program $< verify reset exit"

clean:
	rm -rf build

.PHONY: all clean flash flash-ocd

# ---- host simulation of the control code against a hub-motor model ----
SIM_SRC = sim/sim.c src/motor.c src/hall.c src/mathx.c src/config.c src/cli.c
build/sim: $(SIM_SRC) src/*.h sim/include/stm32g4xx.h | build
	gcc -O2 -g -std=gnu11 -Wall -Wextra -DSIM -Isim/include -Isrc $(SIM_SRC) -lm -o $@

sim: build/sim
	./build/sim

.PHONY: sim
