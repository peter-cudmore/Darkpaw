#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include "servos.h"
#include "led.h"
#include "camera.h"
#include "estimators.h"

int main(int arvc, char* argv[]){
    
    int exit_code = 0;
    // startup routine
    if ((exit_code = init_leds()) < 0)       { goto total_failure; }
    if ((exit_code = init_servos()) < 0)     { goto shutdown_leds; }
    if ((exit_code = init_camera()) < 0)     { goto shutdown_servos; }
    if ((exit_code = init_estimators()) < 0) { goto shutdown_camera; }
    // web api
    // bluetooth interface
    


// shutdown routime
shutdown_camera:    close_camera();
shutdown_servos:    close_servos();
shutdown_leds:      close_leds();
total_fail:         return exit_code;
} 
