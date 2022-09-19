
CC=xc16-gcc
PYTHON=python
FLAGS := -mcpu=33CK64MC103 -Wall -g
CFLAGS := -menable-fixed=truncation -I/opt/microchip/xc16/v1.70/include -I/opt/microchip/xc16/v1.70/support/dsPIC33C/h
LINKERSCRIPT = p33CK64MC103.gld

out: foc-asm.o foc.o sin_table.o
	$(CC) $(FLAGS) $^ -T $(LINKERSCRIPT) -o $@

foc-asm.o: foc-asm.S
	$(CC) $(FLAGS) -c $< -o $@

foc.s: foc.c
	$(CC) $(FLAGS) $(CFLAGS) -S $< -o $@

foc.o: foc.c
	$(CC) $(FLAGS) $(CFLAGS) -c $< -o $@

sin_table.o: sin_table.c
	$(CC) $(FLAGS) $(CFLAGS) -c $< -o $@

sin_table.c: sin-table.py
	$(PYTHON) $< > $@

clean:
	rm -rf foc-asm.o foc.o out

.PHONY: clean
