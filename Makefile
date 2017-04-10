.PHONY: all clean

all: may-music-vs1053b-plugin.h
clean:
	del *.o may-music-vs1053b-plugin.h

rom1053b.o: rom1053b.txt
	mkabs -o $@ -f $<

VCC = vcc
VCCFLAGS = -P130 -O2 -fauto-to-static -fcase-in-i -g -v -fsmall-code
INCDIR = -I../VSIDE/libvs1053b
LIBDIR = -L../VSIDE/libvs1053b

.c.o:
	$(VCC) $(VCCFLAGS) $(INCDIR) -o $@ $<

.s.o:
	vsa -v $(INCDIR) -o $@ $<

may-music.o: may-music.s

OBJS = may-music.o rom1053b.o
may-music-vs1053b-plugin.bin: $(OBJS)
	vslink -m user_vs1053.mem $(OBJS) -o $@ $(LIBDIR) -lc

may-music-vs1053b-plugin.dis: may-music-vs1053b-plugin.bin
	vsomd $< > $@

may-music-vs1053b-plugin.h: may-music-vs1053b-plugin.bin
	coff2allboot -o $@ -i plugin $<
