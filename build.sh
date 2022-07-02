#!/bin/sh
rm system.o
#verilator -O3 --top-module limn2600_System --cc rtl/system.v >log.txt || exit
iverilog rtl/system.v -o system.o || exit
vvp system.o >log.txt || exit
