echo
======

A UDP echo server coded using `FreeRTOS` and `lwIP` on the STM32F4XX.

### File structure

```bash
.
├── FreeRTOSConfig.h
├── inc
│   ├── FreeRTOSConfig.h.default
│   ├── lwipopts.h
│   ├── main.h
│   ├── netconf.h
│   ├── serial_debug.h
│   ├── stm32f4x7_eth_bsp.h
│   ├── stm32f4xx_conf.h
│   └── stm32f4xx_it.h
├── main.ld
├── Makefile
├── README.md
├── src
│   ├── main.c
│   ├── netconf.c
│   ├── serial_debug.c
│   ├── stm32f4x7_eth_bsp.c
│   ├── stm32f4xx_it.c
│   ├── system_stm32f4xx.c
│   ├── tcpecho.c
│   └── udpecho.c
├── stm32f427.ld
├── stm32f4x7_eth_conf.h
└── stubs.c

```

### Brief description of the files:
* `FreeRTOSConfig.h` FreeRTOS configuration
* `inc/` headers
* `src`  sources
* `stm32f427.ld` linker file (used instead of `main.ld`)
* `stubs.c` file needed (libc aspects)