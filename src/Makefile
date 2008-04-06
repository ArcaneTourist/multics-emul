include Makefile.defs


multics_objs: hw6180_cpu.o hw6180_sys.o opcode_text.o misc.o opu.o bits.o apu.o scu.o iom.o mt.o math.o console.o

hw6180_cpu.o: *.h
hw6180_sys.o: *.h
opcode_text.o: *.h
misc.c: *.h
opu.o: *.h
bits.o: *.h
apu.o: *.h
scu.o: *.h
iom.o: *.h
mt.o: *.h
math.o: *.h
misc.o: *.h
console.o: *.h

wc:
	wc hw6180_cpu.c hw6180_sys.c opcode_text.c misc.c opu.c bits.c apu.c scu.c iom.c mt.c math.c console.c hw6180.h

bit-test.o: *.h
bit-test: bit-test.o bits.o
scan-tape: scan-tape.o misc.o bits.o opcode_text.o
mtst: mtst.o math.o
	$(CC) $(CFLAGS) $(LDFLAGS) -lgmp -o $@ mtst.o math.o 
tst: tst.o misc.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ tst.o misc.o

clean:
	rm -f *.o
	rm -f bits bit-test tst test-op scan-tape

opcodes.h: opcodes0.txt opcodes1.txt
	./opcodes2c.pl | sed -ne '/^#ifndef/q' -e p > .tmpf && mv .tmpf $@
opcode_text.c: opcodes0.txt opcodes1.txt
	./opcodes2c.pl | sed -ne '/^#ifndef/,$$p' > .tmpf && mv .tmpf $@
