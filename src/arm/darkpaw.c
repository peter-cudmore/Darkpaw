#include "../darkpaw.h"
#include "pigpio/pigpio.h"
#include "camera.h"
#include "servos.h"
#include "led.h"
#include "camera.h"
#include "sensors.h"

static const unsigned stages = 3;

static void* shutdown_sequence[stages];
static unsigned startup_stage = stages;

int initialise() {
    
    printf("Initialising GPIO....\n");
    if ((exit_code = gpioInitialise()) == 0) { return -startup_stage; }
    else { shutdown_sequence[--startup_stage] = gpioTerminate;}
    printf("Done\n");
    
    printf("Initialising Leds...\n");
    if ((exit_code = init_leds()) < 0) { return -startup_stage; }
    else { shutdown_sequence[--startup_stage] = close_leds; }
    printf("Done\n");

    printf("Initialising Servos...\n");
    if ((exit_code = init_servos()) < 0) { goto shutdown_leds; }
    else { shutdown_sequence[--startup_stage] = close_servos; }
    printf("Done\n");

    // sensors (mpu6050)
    // if ((exit_code = init_camera()) < 0)     { goto shutdown_servos; }
    // if ((exit_code = init_estimators()) < 0) { goto shutdown_camera; }
    // web api
    // bluetooth interface
    //
    //

    printf("Startup Complete\n");
    return startup_stage;


    // shutdown routime
};

void shutdown() {
    for (unsigned i = startup_stage; i < stages; i++) {
        shutdown_seqeuence[i]();
    }
};

