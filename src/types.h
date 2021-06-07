#ifndef TYPES_H
#define TYPES_H
#include <cglm/cglm.h>
#include <stdbool.h>

typedef unsigned char u8;
typedef unsigned short u16;

struct LegState {
    vec3 position;      // In body frame
    vec3 joint_angles;
    u16 motor_pwm[3];
    bool is_raised;
};

enum LegEnum {
    FrontLeft  = 0,
    RearLeft,
    FrontRight,
    RearRight
};


struct DarkpawState {
    struct LegState legs[4];
};


#endif // TYPES_H
