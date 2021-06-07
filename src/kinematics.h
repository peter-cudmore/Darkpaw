//
// Created by Pete on 6/06/2021.
//

#ifndef DARKPAW_KINEMATICS_H
#define DARKPAW_KINEMATICS_H
#include "types.h"

bool forward_leg_kinematics(vec3 motor_angles, vec3 position_out);
bool inverse_leg_kinematics(vec3 position, vec3 motor_angles_out);

#endif //DARKPAW_KINEMATICS_H
