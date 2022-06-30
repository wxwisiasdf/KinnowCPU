rm system.o
iverilog system.v -o system.o || exit
vvp system.o >log.txt || exit
