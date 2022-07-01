![Title](https://raw.githubusercontent.com/wxwisiasdf/KinnowCPU/main/Logo.png)

![CI](https://github.com/wxwisiasdf/KinnowCPU/actions/workflows/auto.yaml/badge.svg?branch=main)

CPU implementing the Limn2600 architecture.

## Features
* [x] ISA implemented
* [x] Pipelining
* [x] Parallel fetch and execution
* [ ] Multicore support
* [ ] Memory scheduling
* [ ] Cache
* [ ] Paging
* [ ] TLB cache
* [ ] Branch prediction

# Boot ROM
Run this with your binary, non LOFF boot ROM - see the SDK for the A3X bootloader:
```sh
xxd -ps -e -c 4 src/boot.bin | awk '{print $2}' >rom.txt
```
It will then be loaded into 0xFFFE0000 and mirrored by the DRAM controller.
