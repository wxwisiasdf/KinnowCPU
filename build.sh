#!/bin/sh
[ ! -f system.o ] || rm system.o
verilator \
    --top-module l2k_cpu -I./rtl -I./rtl/kinnow \
    -O3 --cc --Wall -Wno-fatal rtl/l2k_cpu.sv || exit

clang++ -g -Og -Wall -D__LIBRETRO__ -fPIC -fPIC -c -I obj_dir -I /usr/share/verilator/include -o libretro.o libretro.cpp || exit
clang++ -g -fPIC -shared -Wl,--version-script=./link.T -Wl,--no-undefined  -o kinnowcpu_libretro.so ./libretro.o -lm || exit

retroarch -L $PWD/kinnowcpu_libretro.so --verbose boot.bin 1>log.txt || exit

#cd obj_dir
#./Vl2k_soc >log.txt
#cd ..
