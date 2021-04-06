#ifndef MODEL_H
#define MODEL_H

#include <stdbool.h>
#include "types.h"

#define MOTOR_ZERO 300

typedef struct {
    u16 pwm;
    float value;
    float grad;
} AngleLookupTableItem;

typedef struct {
    int size;
    AngleLookupTableItem entries[401];
} AngleLookupTable;

float* get_leg_position(float* angles, float* position);
float** get_leg_jacobian(float* angles, float** jacobian);
float get_jacobian_determinant(float* angles);


// return -1 if not found, otherwise the index
int lookup_by_pwm(AngleLookupTable* table, int pwm);
int lookup_by_value(AngleLookupTable* table, float value);

bool forward_leg_kinematics(u16* pwm_values, float* position);
bool inverse_leg_kinematics(float* x_desired, u16* pwm_values, float tol, int max_steps);
#endif