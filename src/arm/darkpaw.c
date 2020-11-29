#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "../darkpaw.h"
#include "../model.h"
#include "pigpio/pigpio.h"
#include "camera.h"
#include "servos.h"
#include "led.h"
#include "camera.h"
#include "sensors.h"

#define stages 4

void (*shutdown_sequence[stages])(void);
unsigned startup_stage = stages;
struct Darkpaw *darkpaw;

void free_darkpaw_model(void){
	if (darkpaw) {free(darkpaw);}
}


int initialise() {
    
    int exit_code;
    printf("Initialising GPIO....\n");
    if ((exit_code = gpioInitialise()) == 0) { return startup_stage; }
    else { shutdown_sequence[--startup_stage] = gpioTerminate;}
    printf("Done\n");
    
    printf("Initialising Leds...\n");
    if ((exit_code = init_leds()) < 0) { return startup_stage; }
    else { shutdown_sequence[--startup_stage] = close_leds; }
    printf("Done\n");

    printf("Initialising Servos...\n");
    if ((exit_code = init_servos()) < 0) { return startup_stage; }
    else { shutdown_sequence[--startup_stage] = close_servos; }
    printf("Done\n");

    // sensors (mpu6050)
    // if ((exit_code = init_camera()) < 0)     { goto shutdown_servos; }
    // if ((exit_code = init_estimators()) < 0) { goto shutdown_camera; }
    // web api
    // bluetooth interface
    //
    //
    printf("Allocating Model Interface..\n");
    if ((darkpaw = new_model()) == NULL){ return startup_stage;}
    else { shutdown_sequence[--startup_stage] = free_darkpaw_model;}   
    printf("Done\n");
    printf("Startup Complete\n");

    return 0;
};


void shutdown() {
    for (unsigned i = startup_stage; i < stages; i++) {
        shutdown_sequence[i]();
    }
};

#define MAX(a, b) a > b ? a : b 
#define WRAP(a) a > M_2_PI ? a - M_2_PI : a
// 1 -> 3 -> 2 -> 4 
const float leg_rest_x = 65.603f;
const float leg_rest_y = 45.364f;
const float leg_rest_z = -2.9311f;

void step_test(float delta_time){

    static float phase = 0.0f;
    
    const float freq = M_PI;
    
    unsigned motor[3] = { 300, 300, 300};
    vec3 position = {
        leg_rest_x + 5.0f * sinf(phase),
        leg_rest_y + 2,
        leg_rest_z + MAX(2.0f * sinf(phase + M_PI), 0)
    };

    LegAngles angles;
    
    leg_position_to_angles(FrontLeft, position, &angles, motor);
    
    set_motor_angle(Angle, angles[TorsoServoArm]);
    set_motor_angle(Radius, angles[LegY]);
    set_motor_angle(Height, angles[LegTriangle]);      


    phase = WRAP(phase + freq * delta_time);

};

bool atoi_failed(int result, char* string) {
    return (strcmp(string, "0") != 0 && result == 0)
}
bool atof_failed(float result, char* string) {
    bool is_zero = ((strcmp(string, "0") == 0) || strcmp(string, "0.0") == 0);
    return is_zero && (result != 0.0f);
}

bool repl_loop() {
    char buffer[80];
    char* cmd;
    char *sep = " ";
    char* leg, *param_1, *param_2, *param_3;
    int leg_int;
    // commands
    // pwm leg motor value
    // angle leg joint angle
    // pos leg dx dy dz

    while (fgets(buffer, 80, stdin) != NULL) {

        cmd = strtok(buffer, sep);
        leg = strtok(NULL, sep);
        param_1 = strtok(NULL, sep);
        param_2 = strtok(NULL, sep);
        param_3 = strtok(NULL, sep);

        if (cmd == NULL) {
            if (strcmp(buffer, "quit") == 0) {
                return true;
            }
            continue;
        }

        if ((leg == NULL) || ((leg_int = atoi(leg) == 0) && (strcmp(leg, "0") != 0)) || (leg_int < 0) || (leg_int > 3))
        {
            printf("Invalid Command - Leg not spefied\n");
            continue;
        }
        if ((param_1 == NULL) || (param_2 == NULL)) {
            printf("Invalid command - not enough parameters\n");
        }
        else if ((strcmp(cmd, "pwm") == 0)) {
            int motor = atoi(param_1);
            int value = atoi(param_2);
            if (atoi_failed(motor, param_1) || atoi_failed(value, param_2) || (value < SERVO_MIN) || (value > SERVO_MAX)) { printf("Invalid Command\n"); }
            else {
                unsigned channel = (motor + leg_int * 4);
                unsigned off = value;
                printf("Setting channel %u to %u (leg: %i motor: %i)...", channel, value, leg_int, motor);
                if (set_pwm(channel, 0, off) != -1) {
                    printf("done\n");
                }
                else { 
                    printf("failed\n");
                }
                
            }
        }
        else if (strcmp(cmd, "angle") == 0)
        {
            int joint = atoi(param_1);
            int degs = atoi(param_2);

            if (atoi_failed(joint, param_1) || atoi_failed(degs, param_2) || (joint < 0) || (joint > 3) || (degs < 0) || degs > 360) {
                printf("Could not set angle\n");
            } else {
                float rads = (float)degs * M_2_PI / 360.0f;
                unsigned channel = leg_int * 4 + joint;
                
                printf("Setting channel %u to angle: %i (leg: %i motor: %i)...", channel, rads, leg_int, joint);
                if (set_motor_angle(channel, rads)!= -1) {
                    printf("done\n");
                }
                else {
                    printf("failed\n");
                }
            }
        }
        else if ((strcmp(cmd, "pos") == 0) && (param_3!=NULL)) {

            float  x = atof(param_1);
            float y = atof(param_2);
            float z = atof(param_3);
            if (atof_failed(x, param_1) || atof_failed(y, param_2) || atof_failed(z, param_3)) {
                printf("Invalid command: could not convert to positon");
            }
            else {
                vec3 position = {
                    leg_int % 2 == 0 ? x + leg_rest_x : x - leg_rest_x,
                    leg_int < 2 ? y + leg_rest_y : y - leg_rest_y,
                    leg_rest_z
                };
                unsigned motor[3] = { 0, 0, 0 };
                LegAngles angles;

                if (leg_position_to_angles(leg_int, position, &angles, motor)== true) {
                    set_motor_angle(Angle, angles[TorsoServoArm]);
                    set_motor_angle(Radius, angles[LegY]);
                    set_motor_angle(Height, angles[LegTriangle]);
                    printf("Setting angles to (%f, %f,%f)", angles[TorsoServoArm], angles[LegY], angles[LegTriangle]);

                }  else {
                    printf("Could not solve angles for positons\n");
                };
            }
        }
        else {
            printf("Invalid Command\n");
        }
    }
}