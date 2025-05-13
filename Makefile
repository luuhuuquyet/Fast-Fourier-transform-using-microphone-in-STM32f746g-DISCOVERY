# Makefile for STM32F746NGHx project
# Generated from ES.uvprojx

# Toolchain
CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size

# Project name
PROJECT = FFT-STM32746G

# Build directories
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj

# Source files
C_SOURCES = \
Src/main.c \
Src/stm32f7xx_it.c \
Src/stm32f7xx_hal_msp.c \
Src/stm32f7xx_hal_timebase_tim.c \
Src/system_stm32f7xx.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_adc.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_adc_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_exti.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_crc.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_crc_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dcmi.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dcmi_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma2d.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_fmc.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_sdram.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_ltdc.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_ltdc_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dsi.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_qspi.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rtc.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rtc_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_sai.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_sai_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_sdmmc.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_sd.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spdifrx.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spi.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spi_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_hcd.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_usb.c \
Drivers/BSP/STM32746G-Discovery/stm32746g_discovery.c \
Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_audio.c \
Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_lcd.c \
Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_sdram.c \
Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_ts.c \
Drivers/BSP/Components/ft5336/ft5336.c \
Drivers/BSP/Components/wm8994/wm8994.c \
Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_f32.c \
Drivers/CMSIS/DSP/Source/TransformFunctions/arm_rfft_fast_f32.c \
Drivers/CMSIS/DSP/Source/TransformFunctions/arm_rfft_fast_init_f32.c \
Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix8_f32.c \
Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_f32.c \
Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c

ASM_SOURCES = \
	startup_stm32f746xx.s \
	Drivers/CMSIS/DSP/Source/TransformFunctions/arm_bitreversal2.S


# Object files
C_OBJECTS = $(C_SOURCES:%.c=$(OBJ_DIR)/%.o)
ASM_OBJECTS = $(ASM_SOURCES:%.s=$(OBJ_DIR)/%.o)
OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

# Include paths
INCLUDE_PATHS = \
-IInc \
-IDrivers/STM32F7xx_HAL_Driver/Inc \
-IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F7xx/Include \
-IDrivers/CMSIS/Include \
-IDrivers/BSP/STM32746G-Discovery \
-IDrivers/CMSIS/DSP/Include

LIBS = -lc -lm -lnosys 
LIBS += -L./Drivers/CMSIS/Lib/GCC/ -l:libarm_cortexM7l_math.a -l:libarm_cortexM7lfdp_math.a -l:libarm_cortexM7lfsp_math.a

# Compiler flags
CFLAGS = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard \
         -Wall -O0 -g -std=gnu11 \
         -DUSE_HAL_DRIVER -DSTM32F746xx -DARM_MATH_CM7 -D__FPU_PRESENT=1 -D__FPU_USED=1 \
         $(INCLUDE_PATHS)

ASFLAGS = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -g

# Linker flags
LDFLAGS = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard \
          -TSTM32F746NGHx_FLASH.ld -Wl,--gc-sections \
          -specs=nano.specs -specs=nosys.specs

# Targets
all: $(BUILD_DIR)/$(PROJECT).elf $(BUILD_DIR)/$(PROJECT).hex $(BUILD_DIR)/$(PROJECT).bin

$(BUILD_DIR)/$(PROJECT).elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) $(LIBS) -o $@
	$(SIZE) $@

$(BUILD_DIR)/$(PROJECT).hex: $(BUILD_DIR)/$(PROJECT).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR)/$(PROJECT).bin: $(BUILD_DIR)/$(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

$(OBJ_DIR)/%.o: %.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/%.o: %.s
	@mkdir -p $(@D)
	$(AS) $(ASFLAGS) $< -o $@

clean:
	rm -rf $(BUILD_DIR)

flash: $(BUILD_DIR)/$(PROJECT).bin
	st-flash write $(BUILD_DIR)/$(PROJECT).bin  0x08000000

.PHONY: all clean flash