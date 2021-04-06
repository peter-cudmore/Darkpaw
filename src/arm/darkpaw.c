#include <stdio.h>
#include <math.h>

#include "darkpaw.h"
#include "servos.h"
#include "led.h"
#include "../model.h"
#include "pigpio/pigpio.h"
#include "../walk_phase.h"
#include <cglm/cglm.h>

u16 motor_values[12] = {
    MOTOR_ZERO, MOTOR_ZERO, MOTOR_ZERO,
    MOTOR_ZERO, MOTOR_ZERO, MOTOR_ZERO,
    MOTOR_ZERO, MOTOR_ZERO, MOTOR_ZERO,
    MOTOR_ZERO, MOTOR_ZERO, MOTOR_ZERO
};

WalkStatus legs[4];
vec3 x[4];
float t;

int initialise(){
    if (!gpioInitialise()) {
        return  -1;
    }
    if (init_leds() != 0){
        return -1;
    }
    if (init_servos() != 0){
        fprintf(stderr, "Failed to start servos\n");
        return -1;

    }
    if (set_pwms(motor_values) !=0 ){
        fprintf(stderr, "Failed to set rest positions\n");
        return -1;
    }

    if (!forward_leg_kinematics(motor_values, x[0])){
        fprintf(stderr, "Could not solve forward kinematics at rest\n");
        return -1;
    }
    t = 0;
    return 0;
}

float next_update = 0.0f;

void tick(float delta_time){

    t += delta_time;
    // run a pick up and move the leg phase


}


void shutdown(){
    close_servos();
    close_leds();
    gpioTerminate();
}


