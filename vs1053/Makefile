.PHONY: all clean

all: may-music-vs1053b-plugin.h may-music-vs1053b-plugin.dis
clean:
	del *.o may-music-vs1053b-plugin.h mem_desc.available

rom1053b.o: rom1053b.txt
	mkabs -o $@ -f $<

VCC = vcc
VCCFLAGS = -P130 -O2 -fauto-to-static -fcase-in-i -g -v -fsmall-code
INCDIR = -I../VSIDE/libvs1053b
LIBDIR = -L../VSIDE/libvs1053b
MEM_FILE = user_vs1053.mem

.c.o:
	$(VCC) $(VCCFLAGS) $(INCDIR) -o $@ $<

.s.o: $(MEM_FILE)
	vsa -v $(INCDIR) -o $@ $<

may-music.o: may-music.s

OBJS = may-music.o rom1053b.o
may-music-vs1053b-plugin.bin: $(MEM_FILE) $(OBJS)
	vslink -m $(MEM_FILE) $(OBJS) -o $@ $(LIBDIR) -lc

may-music-vs1053b-plugin.dis: may-music-vs1053b-plugin.bin
	vsomd -o $@ $<

may-music-vs1053b-plugin.h: may-music-vs1053b-plugin.bin
	coff2allboot -o $@ -i plugin $<
