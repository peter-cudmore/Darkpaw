#include "darkpaw.h"
#include "servos.h"
#include "led.h"
#include "model.h"

u16 motor_values[12];


int initialise(){
    if (!init_leds()){
        return -1;
    }
    if (!init_servos()){
        return -1;
    }
}

void shutdown(){
    close_servos();
    close_leds();
}


