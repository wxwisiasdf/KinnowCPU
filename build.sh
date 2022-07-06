#!/bin/sh
rm system.o
verilator \
    --top-module limn2600_System \
    -O3 --cc --exe --build \
    main.cpp \
    rtl/system.sv \
    -CFLAGS "$(sdl2-config --cflags)" \
    -LDFLAGS "$(sdl2-config --libs)" || exit

cd obj_dir
./Vlimn2600_System >log.txt
cd ..
