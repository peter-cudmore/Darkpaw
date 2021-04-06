#include "darkpaw.h"
#include "servos.h"
#include "led.h"
#include "model.h"
#include "pigpio/pigpio.h"

u16 motor_values[12];


int initialise(){
    if (!gpioInitialise()) {
        return  -1;
    }
    if (init_leds() != 0){
        return -1;
    }
    if (init_servos() != 0){
        return -1;
    }
}

void shutdown(){
    close_servos();
    close_leds();
    gpioTerminate();
}


