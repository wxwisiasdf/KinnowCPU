# limine2600 verilog
Run this with your binary, non LOFF boot ROM:
```sh
xxd -ps -e -c 4 src/boot.bin | awk '{print $2}' >rom.txt
```
It will then be loaded into 0xFFFE0000 and mirrored by the DRAM controller.
