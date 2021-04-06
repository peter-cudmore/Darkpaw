#ifndef SERVOS_H
#include "types.h"
#include <stdbool.h>

int   init_servos(void);
void  close_servos(void);
bool  set_motor_angle(unsigned motor, float radians);
int set_pwm(unsigned channel, u16 on, u16 off);
#define SERVO_MAX  550
#define SERVO_MIN  100
#define SERVO_ZERO 300


#define SERVOS_H
#endif
