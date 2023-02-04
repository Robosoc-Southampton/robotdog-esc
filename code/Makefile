
CC=xc16-gcc
PYTHON=python
FLAGS := -mcpu=33CK64MC103 -Wall -g
CFLAGS := -menable-fixed=truncation -I/opt/microchip/xc16/v1.70/include -I/opt/microchip/xc16/v1.70/support/dsPIC33C/h
LINKERSCRIPT = p33CK64MC103.gld

COBJS := adc.o pwm.o hall.o i2c.o
COBJS_NO_H := foc.o sin_table.o

out: foc-asm.o $(COBJS) $(COBJS_NO_H)
	$(CC) $(FLAGS) $^ -T $(LINKERSCRIPT) -o $@

foc-asm.o: foc-asm.S
	$(CC) $(FLAGS) -c $< -o $@

$(COBJS): %.o: %.c %.h utils.h
	$(CC) $(FLAGS) $(CFLAGS) -c $< -o $@

$(COBJS_NO_H): %.o: %.c
	$(CC) $(FLAGS) $(CFLAGS) -c $< -o $@

# adc.o: adc.c
# 	$(CC) $(FLAGS) $(CFLAGS) -c $< -o $@

# sin_table.o: sin_table.c
# 	$(CC) $(FLAGS) $(CFLAGS) -c $< -o $@

sin_table.c: sin-table.py
	$(PYTHON) $< > $@

clean:
	rm -rf foc-asm.o $(COBJS) out

.PHONY: clean
