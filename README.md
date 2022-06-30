# limn2600 verilog
Run this with your binary, non LOFF boot ROM:
```sh
xxd -ps -e -c 4 src/boot.bin | awk '{print $2}' >rom.txt
```
It will then be loaded into 0xFFFE0000 and mirrored by the DRAM controller.

# View schematic
If you like cool graphs that do nothing but make you wonder "I can do better than a commercial grade synthetizer!"
then realize the synthetizer is 5 parallel universes ahead of you. Then follow these steps:

```sh
# install yosys and npm
sudo apt install -y yosys npm
```
