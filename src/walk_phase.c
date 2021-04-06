//
// Created by Pete on 6/04/2021.
//

#include <stdbool.h>
#include <math.h>
#include "types.h"
#include "model.h"
#include <cglm/cglm.h>
#include "walk_phase.h"

vec3 e_z = {0, 0, 1};
mat3 L_z = {{0, 1,0},{-1, 0, 0}, {0, 0, 1}};
mat3 L_zL_z = {{-1,0,0}, {0,-1,0}, {0,0,1}};

#define IK_TOL 0.1
#define IK_ITERS 25

bool next_value(WalkStatus* walkStatus, float t, u16* motor_values){
    // Input:
    //      walkStatus      - walk state
    //      t               - elapsed time
    //      motor_values    - current motor PWM values
    // Output: True if the operation is reachable.


    float s = fmodf(t, walkStatus->duration);

    vec3 temp;
    vec3 position;
    if (s > walkStatus->sweep_duration) {
         float l = (s - walkStatus->sweep_duration) / (walkStatus->duration - walkStatus->sweep_duration);
         glm_vec3_scale(walkStatus->start,l, position);
         glm_vec3_scale(walkStatus->end, 1-l, temp);
         glm_vec3_add(position, temp, position);
         glm_vec3_scale(e_z, walkStatus->max_h *l * (1 - l), temp);
         glm_vec3_add(position, temp, position);
    } else {
        vec3 v = {walkStatus->dx, walkStatus->dy, 0};
        glm_vec3_scale(v, s, position);
        glm_vec3_add(position, walkStatus->start, position);
        if (walkStatus->omega != 0){
            float c1 = (1-cosf(walkStatus->omega *s)) / walkStatus->omega;
            float s1 = sin(walkStatus->omega *s) / walkStatus->omega;
            glm_mat3_mul(L_z, v, temp);
            glm_vec3_scale(temp, c1, temp);
            glm_vec3_add(position, temp, position);
            glm_mat3_mul(L_zL_z, v, temp);
            glm_vec3_scale(temp, s - s1, temp);
            glm_vec3_add(position,temp,position);
        }
    }
    return inverse_leg_kinematics(position, motor_values, IK_TOL, IK_ITERS);
};

bool reposition_leg_in_bodyframe(vec3 x_now, vec3 x_final, float duration, WalkStatus* out){

    u16 motor_values[3] = {MOTOR_ZERO, MOTOR_ZERO, MOTOR_ZERO};

    if (!inverse_leg_kinematics(x_final, motor_values, IK_TOL, IK_ITERS)){
        return false; //unreachable.
    }
    glm_vec3_copy(x_now, out->start);
    glm_vec3_copy(x_final, out->end);
    out->sweep_duration = 0.0f;
    out->duration = 1.0f;
    out->max_h = x_now[2] + 20;
    out->dx = 0;
    out->dy= 0;
    out->omega = 0;
}

