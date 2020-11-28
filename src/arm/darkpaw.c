#include <math.h>
#include <stdbool.h>

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

void step_test(float delta_time){

    static float phase = 0.0f;
    
    const float freq = M_PI;
    
    const float leg_rest_x = 65.603f;
    const float leg_rest_y = 45.364f;
    const float leg_rest_z = -2.9311f;
    unsigned motor[3] = { 300, 300, 300};
    vec3 position = {
        leg_rest_x + 5.0f * sinf(phase),
        leg_rest_y + 2,
        leg_rest_z + MAX(2.0f * sinf(phase + M_PI), 0)
    };

    LegAngles angles;
    
    leg_position_to_angles(FrontLeft, position, &angles, { 0, 0, 0 });
    
    set_motor_angle(Angle, angles[TorsoServoArm]);
    set_motor_angle(Radius, angles[LegY]);
    set_motor_angle(Height, angles[LegTriangle]);      


    phase = WRAP(phase + freq * dt);

};
