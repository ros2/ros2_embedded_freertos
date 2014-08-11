#
#	This makefile produces a simple binary that tests the STM32 peripherals
#

CROSS_COMPILE=arm-none-eabi-
QEMU_STM32 ?= ../../../RTOS/qemu_stm32/arm-softmmu/qemu-system-arm
#QEMU_STM32 ?= qemu-system-arm

ARCH=CM3
VENDOR=ST
PLAT=STM32F10x
CMSIS_LIB=../../rtos/CMSIS/$(ARCH)
STM32_LIB=../../rtos/STM32F10x_StdPeriph_Driver

CMSIS_PLAT_SRC = $(CMSIS_LIB)/DeviceSupport/$(VENDOR)/$(PLAT)

FREERTOS_SRC = ../../rtos/FreeRTOS
FREERTOS_INC = $(FREERTOS_SRC)/include/                                       
FREERTOS_PORT_INC = $(FREERTOS_SRC)/portable/GCC/ARM_$(ARCH)/

all: peripherals

peripherals: main.c
	$(CROSS_COMPILE)gcc \
		-Wl,-Tmain.ld -nostartfiles \
		-I. -I$(FREERTOS_INC) -I$(FREERTOS_PORT_INC) \
		-I../../rtos/CMSIS/CM3/CoreSupport \
		-I../../rtos/CMSIS/CM3/DeviceSupport/ST/STM32F10x \
		-I../../rtos/STM32F10x_StdPeriph_Driver/inc \
		-fno-common -O0 \
		-gdwarf-2 -g3 \
		-mcpu=cortex-m3 -mthumb \
		-o main.elf \
		\
		$(CMSIS_LIB)/CoreSupport/core_cm3.c \
		$(CMSIS_PLAT_SRC)/system_stm32f10x.c \
		$(CMSIS_PLAT_SRC)/startup/gcc_ride7/startup_stm32f10x_md.s \
		$(STM32_LIB)/src/stm32f10x_rcc.c \
		$(STM32_LIB)/src/stm32f10x_gpio.c \
		$(STM32_LIB)/src/stm32f10x_usart.c \
		$(STM32_LIB)/src/stm32f10x_exti.c \
		$(STM32_LIB)/src/misc.c \
		\
		$(FREERTOS_SRC)/croutine.c \
		$(FREERTOS_SRC)/list.c \
		$(FREERTOS_SRC)/queue.c \
		$(FREERTOS_SRC)/tasks.c \
		$(FREERTOS_SRC)/portable/GCC/ARM_CM3/port.c \
		$(FREERTOS_SRC)/portable/MemMang/heap_1.c \
		\
		stm32_p103.c \
		main.c
	$(CROSS_COMPILE)objcopy -Obinary main.elf main.bin
	$(CROSS_COMPILE)objdump -S main.elf > main.list

qemu: main.bin $(QEMU_STM32)
	$(QEMU_STM32) -M stm32-p103 -kernel main.bin

qemudbg: main.bin $(QEMU_STM32)
	$(QEMU_STM32) -M stm32-p103 \
		-gdb tcp::3333 -S \
		-kernel main.bin

gdb: main.bin $(QEMU_STM32)
	$(QEMU_STM32) -M stm32-p103 \
		-gdb tcp::3333 -S \
		-kernel main.elf

qemuauto: main.bin
	bash emulate.sh main.bin

emu: main.bin
	bash emulate.sh main.bin

clean:
	rm -f *.elf *.bin *.list
