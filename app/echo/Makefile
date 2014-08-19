#
#	This makefile produces a binary that creates a UDP echo server
#	using lwIP network stack.
#

# Cross compiler
CROSS_COMPILE		=arm-none-eabi-
# Location of the simulator compiled with the right arch
QEMU_STM32 			?= ../../../RTOS/qemu_stm32/arm-softmmu/qemu-system-arm
# Processor arch: Cortex-M3
ARCH 				=CM3
# Vendor is STMicroelectronics
VENDOR 				=ST
# Platform: STM32F427
PLAT 				=STM32F4x7
# Location of several libraries:
CMSIS_LIB 			=../../rtos/CMSIS/STM32F4x7
STM32_LIB 			=../../rtos/STM32F4xx_StdPeriph_Driver
CMSIS_PLAT_SRC 		= $(CMSIS_LIB)/Device/ST/STM32F4xx/Source/Templates
STM32_ETH_LIB 		= ../../rtos/STM32F4x7_ETH_Driver
FREERTOS_SRC 		= ../../rtos/FreeRTOS
FREERTOS_INC 		= $(FREERTOS_SRC)/include/                                       
FREERTOS_PORT_INC 	= $(FREERTOS_SRC)/portable/GCC/ARM_$(ARCH)/
APP_SRC 			= src

# Network stack
# 	choose either the MINIMAL, SEQUENTIAL or COMPLETE API
LWIP 				= ../../rtos/lwip-1.4.1

LWIP_C_SOURCES_MINIMAL_API = \
$(LWIP)/src/api/err.c \
$(LWIP)/src/core/init.c \
$(LWIP)/src/core/mem.c \
$(LWIP)/src/core/memp.c \
$(LWIP)/src/core/netif.c \
$(LWIP)/src/core/pbuf.c \
$(LWIP)/src/core/stats.c \
$(LWIP)/src/core/udp.c \
$(LWIP)/src/core/ipv4/icmp.c \
$(LWIP)/src/core/ipv4/inet.c \
$(LWIP)/src/core/ipv4/ip.c \
$(LWIP)/src/core/ipv4/ip_addr.c \
$(LWIP)/src/core/ipv4/ip_frag.c \
$(LWIP)/src/netif/etharp.c \

LWIP_C_SOURCES_SEQUENTIAL_API = \
$(LWIP)/src/api/netifapi.c \
$(LWIP)/src/api/err.c \
$(LWIP)/src/core/dhcp.c \
$(LWIP)/src/core/init.c \
$(LWIP)/src/core/mem.c \
$(LWIP)/src/core/memp.c \
$(LWIP)/src/core/netif.c \
$(LWIP)/src/core/pbuf.c \
$(LWIP)/src/core/raw.c \
$(LWIP)/src/core/stats.c \
$(LWIP)/src/core/udp.c \
$(LWIP)/src/core/def.c \
$(LWIP)/src/core/ipv4/autoip.c \
$(LWIP)/src/core/ipv4/icmp.c \
$(LWIP)/src/core/ipv4/igmp.c \
$(LWIP)/src/core/ipv4/inet.c \
$(LWIP)/src/core/ipv4/ip.c \
$(LWIP)/src/core/ipv4/ip_addr.c \
$(LWIP)/src/core/ipv4/ip_frag.c \
$(LWIP)/src/core/snmp/asn1_dec.c \
$(LWIP)/src/core/snmp/asn1_enc.c \
$(LWIP)/src/core/snmp/mib2.c \
$(LWIP)/src/core/snmp/mib_structs.c \
$(LWIP)/src/core/snmp/msg_in.c \
$(LWIP)/src/core/snmp/msg_out.c \
$(LWIP)/src/netif/etharp.c \

LWIP_C_SOURCES_COMPLETE_API = \
$(LWIP)/src/api/api_lib.c \
$(LWIP)/src/api/api_msg.c \
$(LWIP)/src/api/err.c \
$(LWIP)/src/api/netbuf.c \
$(LWIP)/src/api/netifapi.c \
$(LWIP)/src/api/sockets.c \
$(LWIP)/src/api/tcpip.c \
$(LWIP)/src/core/dhcp.c \
$(LWIP)/src/core/init.c \
$(LWIP)/src/core/mem.c \
$(LWIP)/src/core/memp.c \
$(LWIP)/src/core/netif.c \
$(LWIP)/src/core/def.c \
$(LWIP)/src/core/pbuf.c \
$(LWIP)/src/core/raw.c \
$(LWIP)/src/core/stats.c \
$(LWIP)/src/core/udp.c \
$(LWIP)/src/core/ipv4/autoip.c \
$(LWIP)/src/core/ipv4/icmp.c \
$(LWIP)/src/core/ipv4/igmp.c \
$(LWIP)/src/core/ipv4/inet.c \
$(LWIP)/src/core/ipv4/ip.c \
$(LWIP)/src/core/ipv4/ip_addr.c \
$(LWIP)/src/core/ipv4/ip_frag.c \
$(LWIP)/src/core/snmp/asn1_dec.c \
$(LWIP)/src/core/snmp/asn1_enc.c \
$(LWIP)/src/core/snmp/mib2.c \
$(LWIP)/src/core/snmp/mib_structs.c \
$(LWIP)/src/core/snmp/msg_in.c \
$(LWIP)/src/core/snmp/msg_out.c \
$(LWIP)/src/netif/etharp.c \

all: app

app: src/main.c
	$(CROSS_COMPILE)gcc \
		-Wl,-Tstm32f427.ld -nostartfiles \
		-I. -I$(FREERTOS_INC) -I$(FREERTOS_PORT_INC) \
		-I$(CMSIS_LIB)/Include \
		-I$(CMSIS_LIB)/Device/ST/STM32F4xx/Include \
		-I../../rtos/STM32F4xx_StdPeriph_Driver/inc \
		-I$(STM32_LIB)/inc \
		-I$(STM32_ETH_LIB)/inc \
		-I$(LWIP)/src/include \
		-I$(LWIP)/src/include/lwip \
		-I$(LWIP)/port/STM32F4x7 \
		-I$(LWIP)/src/include \
		-I$(LWIP)/src/include/ipv4 \
		-I$(LWIP)/src/include/netif \
		-I$(LWIP)/src/include/posix \
		-I$(LWIP)/port/STM32F4x7/FreeRTOS \
		-Iinc \
		-fno-common -O0 \
		-gdwarf-2 -g3 \
		-mcpu=cortex-m3 -mthumb \
		-std=c99 \
		-o main.elf \
		\
		$(CMSIS_PLAT_SRC)/system_stm32f4xx.c \
		$(CMSIS_PLAT_SRC)/gcc_ride7/startup_stm32f4xx.s \
		$(STM32_LIB)/src/stm32f4xx_rcc.c \
		$(STM32_LIB)/src/stm32f4xx_gpio.c \
		$(STM32_LIB)/src/stm32f4xx_usart.c \
		$(STM32_LIB)/src/stm32f4xx_exti.c \
		$(STM32_LIB)/src/stm32f4xx_syscfg.c \
		$(STM32_LIB)/src/misc.c \
		\
		$(FREERTOS_SRC)/croutine.c \
		$(FREERTOS_SRC)/list.c \
		$(FREERTOS_SRC)/queue.c \
		$(FREERTOS_SRC)/tasks.c \
		$(FREERTOS_SRC)/portable/GCC/ARM_CM3/port.c \
		$(FREERTOS_SRC)/portable/MemMang/heap_1.c \
		\
		$(STM32_ETH_LIB)/src/stm32f4x7_eth.c \
		$(LWIP_C_SOURCES_COMPLETE_API) \
		$(LWIP)/port/STM32F4x7/FreeRTOS/sys_arch.c \
		$(LWIP)/src/core/tcp.c \
		$(LWIP)/src/core/timers.c \
		$(LWIP)/src/core/ipv4/inet_chksum.c \
		$(LWIP)/port/STM32F4x7/FreeRTOS/ethernetif.c \
		$(APP_SRC)/netconf.c \
		$(APP_SRC)/serial_debug.c \
		$(APP_SRC)/stm32f4x7_eth_bsp.c \
		$(APP_SRC)/tcpecho.c \
		$(APP_SRC)/udpecho.c \
		stubs.c \
		console.c \
		src/main.c \

	$(CROSS_COMPILE)objcopy -Obinary main.elf main.bin
	$(CROSS_COMPILE)objdump -S main.elf > main.list

qemu: main.bin $(QEMU_STM32)
	$(QEMU_STM32) -M stm32-p103 -kernel main.bin

qemudbg: main.bin $(QEMU_STM32)
	$(QEMU_STM32) -M stm32-p103 \
		-gdb tcp::3333 -S \
		-kernel main.bin

gdb-qemu: main.bin $(QEMU_STM32)
	$(QEMU_STM32) -M stm32-p103 \
		-gdb tcp::3333 -S \
		-kernel main.elf

qemuauto: main.bin
	bash emulate.sh main.bin

emu: main.bin
	bash emulate.sh main.bin

clean:
	rm -f *.elf *.bin *.list

OPENOCD=../../tools/openocd/bin/openocd
program: app
	$(OPENOCD) -f ../../tools/olimex-arm-usb-tiny-h.cfg -f ../../tools/stm32.cfg -c "init; halt; flash write_image erase main.bin 0x8000000; verify_image main.bin 0x8000000; sleep 100; reset run; sleep 100; shutdown"

dump_flash:
	$(OPENOCD) -f ../../tools/olimex-arm-usb-tiny-h.cfg -f ../../tools/stm32.cfg -c "init; halt; dump_image main.bin.dump 0x8000000 0x1000; reset run; shutdown"

gdb_server: app
	$(OPENOCD) -f ../../tools/olimex-arm-usb-tiny-h.cfg -f ../../tools/stm32.cfg -c "init; halt"
	
gdb: app
	$(CROSS_COMPILE)gdb main.elf -x ../../tools/gdb_init_commands

reset:
	$(OPENOCD) -f ../../tools/olimex-arm-usb-tiny-h.cfg -f ../../tools/stm32.cfg -c "init; halt; reset run; shutdown"
