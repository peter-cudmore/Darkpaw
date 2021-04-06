#ifndef MODEL_H
#define MODEL_H
#include <stdbool.h>

typedef struct {
    int pwm;
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


#endif