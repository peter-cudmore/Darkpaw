#include <stdio.h>
#include <math.h>

#include "../darkpaw.h"
#include "servos.h"
#include "led.h"
#include "pigpio/pigpio.h"
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

int initialise(struct DarkpawState* state){
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
    vec3 x_new;
    glm_vec3_copy(x[0], x_new);
    x_new[0] += 40;

    return 0;
}



void read_sensors(struct DarkpawState* state, float delta_time){}
void compute_next_action(struct DarkpawState* state, float delta_time){}
void update_actuators(struct DarkpawState* state, float delta_time){}


void shutdown(){
    close_servos();
    close_leds();
    gpioTerminate();
}


