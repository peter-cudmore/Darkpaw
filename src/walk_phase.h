//
// Created by Pete on 6/04/2021.
//

#ifndef DARKPAW_WALK_PHASE_H
#define DARKPAW_WALK_PHASE_H
#include <cglm/cglm.h>

typedef struct {
    vec3 start;
    vec3 end;
    float dx;
    float dy;
    float omega;
    float max_h;
    float duration;
    float sweep_duration;
} WalkStatus;

bool next_value(WalkStatus* walkStatus, float t, u16* motor_values);
bool reposition_leg_in_bodyframe(vec3 x_now, vec3 x_final, float duration, WalkStatus* out);

#endif //DARKPAW_WALK_PHASE_H
