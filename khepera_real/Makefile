# Makefile para compilar NO robo Khepera IV (Linux Yocto/Poky), com libkhepera.
# Uso:  make            -> gera ./goto_avoid_real
#       make clean      -> remove o binario
CC      = gcc
CFLAGS  = -O2 -Wall
LDLIBS  = -lkhepera -lpthread -lm

all: goto_avoid_real

goto_avoid_real: goto_avoid_real.c
	$(CC) $(CFLAGS) -o $@ goto_avoid_real.c $(LDLIBS)

clean:
	rm -f goto_avoid_real
