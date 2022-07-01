# KinnowCPU

![Title](https://raw.githubusercontent.com/wxwisiasdf/KinnowCPU/main/title.png)

A nice CPU implementing the Limn2600 architecture.

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

Run this with your binary, non LOFF boot ROM:
```sh
xxd -ps -e -c 4 src/boot.bin | awk '{print $2}' >rom.txt
```
It will then be loaded into 0xFFFE0000 and mirrored by the DRAM controller.

# View schematic
If you like cool graphs that do nothing but make you wonder "I can do better than a commercial grade synthetizer!"
then realize the synthetizer is 5 parallel universes ahead of you. Then follow these steps:

```sh
# install yosys
sudo apt install -y yosys
```
