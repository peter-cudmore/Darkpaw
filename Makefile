ARCH = $(shell uname -m)
CC=cc
CFLAGS=-Ideps -Ideps/cglm/include

DIRS = src/ src/tests/
OBJS = model.o darkpaw.o
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

main: $(OBJ) $(MAIN)
	$(CC) -o $@ $^ $(LFLAGS)

$(ALL): $(ODIR)/%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)

test: $(OBJ) $(TEST) 
	$(CC) -o $@ $^ $(LFLAGS)
	./test

kine_test:
	$(CC) src/kinematics.c src/tests/kine_tests.c -o kine_test $(CFLAGS) $(LFLAGS) -Isrc  -DDEBUG
	./kine_test

.PHONY: clean test kine_test

$(ODIR):
	mdkir $(ODIR)

clean:
	-rm -f main
	-rm -f $(ODIR)/*.o
	-rm -f test
	-rm -f kine_test
