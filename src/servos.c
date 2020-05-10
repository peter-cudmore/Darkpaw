#include "servos.h"

int init_servos(void) {
    int result;
    if ((result = gpioInitialise()) < 0)
    {
        // pigpio initialisation failed.
        return result;
    }
    
    // open i2c connection
    // set PWN frequency
    // set pwn initial states
    // set up interface
}

void close_servos(void) {
    gpioTerminate();
}