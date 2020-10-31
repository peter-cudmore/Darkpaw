#ifndef SERVOS_H
#include <stdbool.h>

int   init_servos(void);
void  close_servos(void);
bool  set_motor_angle(unsigned motor, float radians);

#define SERVOS_H
#endif
