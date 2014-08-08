#!/bin/bash

QEMU_STM32=../qemu_stm32/arm-softmmu/qemu-system-arm

emulate () {
	$QEMU_STM32 \
		-M stm32-p103 \
		-gdb tcp::3333 -S \
		-nographic \
		-kernel $1 & qemu_pid=$!
}

stm32_qemu () {
	emulate $1 1> /dev/null
	echo "" > gdb.txt
	arm-none-eabi-gdb -x gdbscript 1> /dev/null & gdb_pid=$!

	echo "Modeling STM32 in QEMU..."
	(sleep $2
	 kill $gdb_pid
	 kill $qemu_pid
	 sleep 1
	 kill -KILL $gdb_pid
	 kill -KILL $qemu_pid
	)& timer=$!
	if ! wait $qemu_pid; then
		kill $timer 2>/dev/null
		echo
		echo "Modeling failed to execute in $2 seconds, giving up."
		exit -1
	else
		perl gdb2vcd.pl gdb.txt
	fi
	kill $timer
}

stm32_qemu $1 5
