include ../Makefile.defs


multics_objs: hw6180_cpu.o hw6180_sys.o opcode_text.o misc.o opu.o

hw6180_cpu.o: *.h
hw6180_sys.o: *.h
opcode_text.o: *.h
misc.c: *.h
opu.o: *.h

clean:
	rm *.o
