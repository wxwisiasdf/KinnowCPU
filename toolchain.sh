#!/bin/bash
export PREFIX="$HOME/opt/cross"
mkdir -p "$PREFIX"
mkdir -p build-gcc && cd build-gcc
[ -f Makefile ] || ../gcc/configure --disable-nls --enable-languages=c,c++ \
    --target=limn2600-mintia --disable-gcov	--disable-multiarch \
    --disable-threads --disable-tls --disable-bootstrap \
    --disable-gnu-unique-object --disable-lto --without-headers \
    --disable-plugin --prefix="$PREFIX" --enable-checking=all || exit
make CC="ccache gcc" CXX="ccache g++" all-gcc -j$(nproc) || exit
if [ "$1" == "install" ]; then
    make CC="ccache gcc" CXX="ccache g++" install-gcc || exit
fi
cd ..
