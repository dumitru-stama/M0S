TOOL=~/armgcc/bin/arm-none-eabi
AFLAGS=-mcpu=cortex-m0

#make sure we keep the .elf files as well for debugging
.PRECIOUS: %.elf %.o

#This will take all ASM files from the current folder and build them
ASMS=$(wildcard *.asm)
BINS=$(ASMS:.asm=.bin)

#list all targets here
all: $(BINS)

%.o: %.asm c
	$(TOOL)-as $(AFLAGS) -o $@ $<

%.elf: %.o
	# Uncomment the line below to compile the full demo
	$(TOOL)-ld -Ttext 0x8000000 $< demo.o -o $@

	# Comment the line below to compile the full demo
	#$(TOOL)-ld -Ttext 0x8000000 $< -o $@
   
%.bin: %.elf   
	$(TOOL)-objcopy -S -O binary $< $@
	$(TOOL)-size $<

gdb: m0s.elf
	$(TOOL)-gdb m0s.elf

c: demo.c
	# no-builtin-free allows me to define my free memory function without warnings
	$(TOOL)-gcc -fno-builtin-free -fno-builtin-memset -mcpu=cortex-m0 -mthumb -c -o2 -Wall demo.c -o demo.o

clean:
	rm -rf *.elf *.o *.bin *.hex *.lst *.map

