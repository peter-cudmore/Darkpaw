#include "../darkpaw.h"
#include "../model.h"
#include "pigpio/pigpio.h"
#include "camera.h"
#include "servos.h"
#include "led.h"
#include "camera.h"
#include "sensors.h"
#include <stdbool.h>
#include <math.h>

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


const float freq = 0.2f;
float domega = M_PI * freq;
float rads = .0f;


void step_test(float delta_time){
	
	unsigned front_left_body = get_servo_index(FrontLeft, Radius);
	unsigned front_right_body = get_servo_index(FrontRight, Radius);
	unsigned rear_left_body = get_servo_index(BackLeft, Radius);
	unsigned rear_right_body = get_servo_index(BackRight, Radius);
//	float rads = 0.25f*M_PI;
	
	if (fabs(rads) > M_PI_2/2){
		domega = -domega;
	}
	rads += domega*delta_time;

	set_motor_angle(front_left_body, rads);
	set_motor_angle(front_right_body, -rads);
	set_motor_angle(rear_right_body, rads);
	set_motor_angle(rear_left_body, -rads);

};
