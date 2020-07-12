#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "pigpio/pigpio.h"
#include "servos.h"
#include "led.h"
#include "camera.h"
#include "estimators.h"

int main(int arvc, char* argv[]) {

    int exit_code = 0;
    // startup routine
    printf("Initialising GPIO....\n");
    if ((exit_code = gpioInitialise()) < 0)               { goto total_fail; }
    printf("Done\n");
    
    printf("Initialising Leds...\n");  
    if ((exit_code = init_leds()) < 0)       { goto shutdown_io; }
    printf("Done\n");

    printf("Initialising Servos...\n");
    if ((exit_code = init_servos()) < 0)     { goto shutdown_leds; }
    printf("Done\n");
    
    // sensors (mpu6050)
    // if ((exit_code = init_camera()) < 0)     { goto shutdown_servos; }
    // if ((exit_code = init_estimators()) < 0) { goto shutdown_camera; }
    // web api
    // bluetooth interface
    
    printf("Complete\n");

// shutdown routime
shutdown_servos:    close_servos();
		    printf("Servo Shutdown Complete\n");
shutdown_leds:      close_leds();
		    printf("LED driver stopped\n");
shutdown_io:        gpioTerminate();
    printf("gpio Terminated\n");

total_fail:         return exit_code;
} 
