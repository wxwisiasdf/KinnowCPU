SDK_PATH := ../sdk

all: build

build: jump.txt copy.txt movb.txt

clean:
	$(RM) *.bin
	$(RM) *.txt

%.bin: %.asm
	$(SDK_PATH)/asm.sh $< $@
	$(SDK_PATH)/link.sh move $@ text=0xFFFE0000,data=text+text_size,bss=0x1000
	$(SDK_PATH)/link.sh binary -nobss $@

%.txt: %.bin
	xxd -ps -e -c 4 $< | awk '{print $$2}' >$@
