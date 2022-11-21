#!/bin/sh
rm system.o
verilator \
    --top-module l2k_soc -I./rtl -I./rtl/kinnow \
    -O3 --cc --exe --build --Wall -Wno-fatal \
    main.cpp \
    rtl/l2k_soc.sv \
    -CFLAGS "$(sdl2-config --cflags)" \
    -LDFLAGS "$(sdl2-config --libs)" || exit

cd obj_dir
./Vl2k_soc >log.txt
cd ..
