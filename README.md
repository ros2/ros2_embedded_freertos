
![](https://avatars3.githubusercontent.com/u/3979232?v=2&s=200)
#ROS 2 for embedded devices

This prototype includes:

| chat|
|----------|
|  tinq dds|
|FreeRTOS|

### File structure:

```.
├── app
├── dds
├── FreeRTOSConfig.h
├── Makefile
├── README.md
└── rtos

```

- the `app` directory contains the final application, in this case the "chat".
- `dds` contains the DDS related aspects
- `rtos` contain the RTOS (FreeDDS) related files

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
