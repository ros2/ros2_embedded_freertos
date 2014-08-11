
![](https://avatars3.githubusercontent.com/u/3979232?v=2&s=200)
#ROS 2 for embedded devices

This prototype includes:

| chat|
|----------|
|  tinq dds|
|FreeRTOS|

### File structure:

```
.
├── app
│   ├── chat
│   └── peripherals
```
the `app` directory contains the final applications.
```
├── dds
│   ├── api
│   ├── apps
│   ├── doc
│   ├── plugins
│   ├── src
│   ├── test
│   └── tools
```
`dds` contains the DDS related aspects
```
└── rtos
```
`rtos` contain the RTOS (FreeDDS) related files, network stack (lwIP), and drivers (Ethernet, peripherals, etc.)
```
    ├── CMSIS
    ├── FreeRTOS
    ├── lwip-1.4.1
    ├── STM32F10x_StdPeriph_Driver
    ├── STM32F4x7_ETH_Driver
    └── STM32F4xx_StdPeriph_Driver


```

### Compile the code

```bash
make
```


### Debugging with gdb:

```bash
make gdb
```

Assuming that `.gdbinit` is configured in your `$HOME` as:

```
# GDB init file
target remote 127.0.0.1:3333
```

In another shell:

```bash
 arm-none-eabi-gdb main.elf
```

![](http://osrfoundation.org/assets/images/osrf_masthead.png)
