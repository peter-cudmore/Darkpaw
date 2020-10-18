ARCH = ($(strip $(uname -m)),)
CC=cc
CFLAGS=-Ideps -Ideps/cglm/include

DIRS = src/
OBJS = main.o model.o darkpaw.o

ifeq ( $(ARCH), arm )
LFLAGS = deps/rpi_ws281x/libws2811.a -lpigpio -lrt
DIRS += src/arm/
OBJS += camera.o led.o sensors.o servos.o
else
DIRS += src/x86_64/
endif
LFLAGS += -lm

vpath %.c $(DIRS)

ODIR = obj

OBJ =  $(patsubst %.o, $(ODIR)/%.o, $(OBJS))

main: $(OBJ) 
	$(CC) -o $@ $^ $(LFLAGS)

$(OBJ): $(ODIR)/%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)


.PHONY: clean

$(ODIR):
	mdkir $(ODIR)

clean:
	-rm -f main
	-rm -f $(ODIR)/*.o
		