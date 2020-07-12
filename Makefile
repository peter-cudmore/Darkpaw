CC=gcc
CFLAGS=-Ideps
LFLAGS = deps/rpi_ws281x/libws2811.a -lpigpio -lrt
ODIR=obj
SDIR=src

_OBJ = camera.o sensors.o led.o main.o servos.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

$(ODIR)/%.o: $(SDIR)/%.c
	$(CC) -c -o $@ $< $(CFLAGS)

main: $(OBJ)
	$(CC) -o $@ $^ $(LFLAGS)

.PHONY: clean

clean:
	rm -f main
	rm -f $(ODIR)/*.o
