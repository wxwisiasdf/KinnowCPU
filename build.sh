#!/bin/sh
rm system.o
iverilog rtl/system.v -o system.o || exit
vvp system.o >log.txt || exit
