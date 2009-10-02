include Makefile.defs


multics.a: hw6180_cpu.o hw6180_sys.o opcode_text.o misc.o opu.o bits.o apu.o eis_mw_desc.o scu.o iom.o mt.o math.o math_real.o console.o symtab.o listing.o
	ar cr $@ $?

wc:
	wc hw6180.h symtab.h bits.h hw6180_cpu.c hw6180_sys.c opcode_text.c misc.c opu.c bits.c apu.c eis_mw_desc.c scu.c iom.c mt.c math.c math_real.c console.c symtab.c listing.c

opcode-count:
	@echo `grep -n 'case opcode._' opu.c | grep -cv ' + 0:'` of `grep -c 'opcode._.*=' opcodes.h`

t: t.c
	gcc -o t t.c

hw6180_cpu.o: *.h
hw6180_sys.o: *.h
opcode_text.o: *.h
misc.o: *.h
opu.o: *.h
bits.o: *.h
apu.o: *.h
eis_mw_desc.o: *.h
scu.o: *.h
iom.o: *.h
mt.o: *.h
math.o: *.h
math_real.o: *.h
console.o: *.h
symtab.o: *.h
listing.o: *.h

bit-test.o: *.h
bit-test: bit-test.o bits.o
scan-tape: scan-tape.o bits.o opcode_text.o
mtst: mtst.o math.o misc.o
	$(CC) $(CFLAGS) $(LDFLAGS) -lgmp -o $@ mtst.o math.o misc.o
tst: tst.o misc.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ tst.o misc.o

clean:
	rm -f *.o multics.a
	rm -f bits bit-test tst test-op scan-tape a.out

opcodes.h: opcodes0.txt opcodes1.txt
	./opcodes2c.pl | sed -ne '/^#ifndef/q' -e p > .tmpf && mv .tmpf $@
opcode_text.c: opcodes0.txt opcodes1.txt
	./opcodes2c.pl | sed -ne '/^#ifndef/,$$p' > .tmpf && mv .tmpf $@

init_early_config.reconstructed.list: init_early_config.damaged.list init_early_config.pl1
	nl -ba init_early_config.pl1 | sed -e 's/^/   /' > .tmp.init_early_config.list
	echo "SOURCE FILES USED" >> .tmp.init_early_config.list
	echo "" >> .tmp.init_early_config.list
	tr -d '\000' < init_early_config.damaged.list | sed -e 's/^14   84 000254/     84 000254/' >> .tmp.init_early_config.list
	mv .tmp.init_early_config.list init_early_config.reconstructed.list
