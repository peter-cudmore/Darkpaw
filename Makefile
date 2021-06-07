ARCH = $(shell uname -m)
PI_ADDRESS = 192.168.1.16

CC=cc
CFLAGS=-Ideps -Ideps/cglm/include -Isrc -g

DIRS = src/ src/tests/
OBJS = darkpaw.o kinematics.o
TEST_OBJS = tests.o
MAIN_OBJS = main.o


LFLAGS = -lm
ifneq ($(filter arm%,$(ARCH) ),)
LFLAGS += deps/rpi_ws281x/libws2811.a -lpigpio -lrt
DIRS += src/arm/
OBJS += camera.o led.o sensors.o servos.o
else
DIRS += src/x86_64/
endif

vpath %.c $(DIRS)

ODIR = obj

OBJ =  $(patsubst %.o, $(ODIR)/%.o, $(OBJS))
MAIN = $(patsubst %.o, $(ODIR)/%.o, $(MAIN_OBJS))
TEST = $(patsubst %.o, $(ODIR)/%.o, $(TEST_OBJS))
ALL = $(OBJ) $(MAIN) $(TEST)

all: main test

main: $(OBJ) $(MAIN)
	$(CC) -o $@ $^ $(LFLAGS)

$(ALL): $(ODIR)/%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)

test: $(OBJ) $(TEST) 
	$(CC) -o $@ $^ $(LFLAGS)

.PHONY: clean test deploy

$(ODIR):
	mdkir $(ODIR)


deploy:
	rsync -ru --exclude-from='rsync_excludes.txt' . pi@$(PI_ADDRESS):~/Darkpaw

clean:
	-rm -f main
	-rm -f $(ODIR)/*.o
	-rm -f test
	-rm -f kine_test
