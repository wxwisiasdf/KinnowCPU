#!/bin/sh
rm Board.o
iverilog Board.v -o Board.o || exit
vvp Board.o >log.txt || exit
