CC=gcc
CFLAGS=-Ideps
LFLAGS = deps/rpi_ws281x/libws2811.a
OBJ = main.o
SRC = src


%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)

main: $(OBJ)
	$(CC) -o $@ $^ $(LFLAGS)

.PHONY: help clean

clean:
	rm main
	rm main.o

help:
	@echo Install

