//
// Created by Pete on 6/06/2021.
//

#include "kinematics.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

float signf(float x){
    if (x > 0)
        return 1.0f;
    if (x < 0)
        return -1.0f;
    return 0;
}


struct LinearApproximation {
    float x0;
    float y0;
    float m;
};

struct CubicApproximation{
    float y0;
    float m;
    float a0;
    float a1;
    float a2;
    float a3;
};

struct Bounds {
    float joint_min;
    float joint_max;
    float motor_min;
    float motor_max;
};

struct Bounds MotorJointBounds[3] = {
        {-0.24817335554500913f,0.330292057264608f,-0.7407434183213392f, M_PI_2 },
        {-0.10203780810552682f,0.544009616053608f, -0.6146594322240899f, M_PI_2},
        {-0.49824629703883416f, 0.5676021229757795f, -M_PI_2, 1.245079362710336f}
};

static const float eps = 0.0001f;


static struct LinearApproximation MotorToJoints1[3] = {
        {0.4150264542367787f, -0.010882568405741644f, 0.25025112115497244f},
        {0.4780684472854033f, 0.16794524292493965, 0.29561221795179893f},
        {-0.16285848204228026f, 0.16794524292493965f, 0.37851401749978886f}
};

static struct CubicApproximation JointsToMotors3[3] = {
        {-0.13835675696667143f, 3.4719409790833065f, 0.5882343222312638, 0.8259042490083175f, -0.09890087821958943f, 0.2729954369131235f},
        {-0.6826278874032411f,3.1103383979841563f,  0.6313674838958347, 0.8139063330874554, -0.0867191933686181,0.22911156119109233},
        {-0.06500012351586663, 1.8765816866899023, -0.19724990295426223, 1.011511069935687,-0.022591958109071986, 0.32850005772747914}
};

static struct CubicApproximation MotorsToJoints3[3] = {
        {-0.35616438356164387, 0.8691749403557391,-0.013868964144305792,0.36574029014217274,0.050433491368451544,-0.07764206890101744 },
        {-0.4347826086956522, 0.9195618934198397,0.16398616034138913,0.39942317950660267, 0.04995421177271449, -0.07715566227172427 },
        {0.11985018726591763, 0.7129187713030218,0.049909075967135466,0.7002918409643484, -0.0024679671452485937,0.16450917388583933}
};

static struct LinearApproximation JointsToMotors1[3] = {
        {-0.010882568405741644f, 0.4150264542367787f,  1.0f / 0.25025112115497244f},
        {0.16794524292493965,0.4780684472854033f,  1.0f / 0.29561221795179893f},
        {0.16794524292493965f, -0.16285848204228026f, 1.0f / 0.37851401749978886f}
};


static float atan_reduce(float coeff_sin, float coeff_cos, float remainder){
    float R = signf(coeff_sin) *  sqrtf(coeff_sin* coeff_sin + coeff_cos*coeff_cos);
    float atan = atan2f(coeff_cos, coeff_sin);

    if ((remainder < eps) && (remainder > -eps)){
        return -atan;
    }
    return -atan - asinf(remainder / R);
}


bool motor_to_joints_linear(vec3 motor_angles, vec3 joints){
    for (int i=0; i<3; i++){
        if ((motor_angles[i] < MotorJointBounds[i].motor_min) || (motor_angles[i] > MotorJointBounds[i].motor_max))
            return false;
        joints[i] = MotorToJoints1[i].m*(motor_angles[i] - MotorToJoints1[i].x0) + MotorToJoints1[i].y0;
    }
    return true;
}

//use cubic approx
bool motor_to_joints(vec3 motor_angles, vec3 joints){
    for (int i=0; i<3; i++){
        if ((motor_angles[i] < MotorJointBounds[i].motor_min) || (motor_angles[i] > MotorJointBounds[i].motor_max))
            return false;
        float x = MotorsToJoints3[i].y0 + MotorsToJoints3[i].m * motor_angles[i];
        joints[i] = MotorsToJoints3[i].a0;
        joints[i] += MotorsToJoints3[i].a1 * x;
        joints[i] += MotorsToJoints3[i].a2 * x * x;
        joints[i] += MotorsToJoints3[i].a3 * x * x * x;
    }
    return true;
}

bool joints_to_motors(vec3 joints, vec3 motors){
    for (int i=0; i<3; i++){
        if ((joints[i] < MotorJointBounds[i].joint_min) || (joints[i] > MotorJointBounds[i].joint_max))
            return false;

        float x = JointsToMotors3[i].y0 + JointsToMotors3[i].m * joints[i] ;
        motors[i] = JointsToMotors3[i].a0;
        motors[i] += JointsToMotors3[i].a1 * x;
        motors[i] += JointsToMotors3[i].a2 * x * x;
        motors[i] += JointsToMotors3[i].a3 * x * x ;
    }
    return true;
}


bool joints_to_motors_linear(vec3 joints, vec3 motors){
    for (int i=0; i<3; i++){
        if ((joints[i] < MotorJointBounds[i].joint_min) || (joints[i] > MotorJointBounds[i].joint_max))
            return false;
        motors[i] = JointsToMotors1[i].m*(joints[i] - JointsToMotors1[i].x0) + JointsToMotors1[i].y0;
    }
    return true;
}


bool forward_leg_kinematics(vec3 motor_values, vec3 position_out){

    vec3 joint_angles;
    if (!motor_to_joints(motor_values, joint_angles))
        return false;

    float cos_wb = cosf(joint_angles[0]);
    float sin_wb = sinf(joint_angles[0]);
    float cos_wr = cosf(joint_angles[1]);
    float sin_wr = sinf(joint_angles[1]);
    float cos_wh = cosf(joint_angles[2]);
    float sin_wh = sinf(joint_angles[2]);

    float r_planar = 75.0f * sin_wr + 32.0f * cos_wh + 15.0f * cos_wr + 66.0f;

    position_out[0] = -sin_wb * r_planar + 42.0f;
    position_out[1] = cos_wb * r_planar + 42.0f;
    position_out[2] = 32.0f * sin_wh + 15.0f * sin_wr - 75.0f * cos_wr;
    return true;
}



bool inverse_leg_kinematics(vec3 position, vec3 motor_angles){

    vec3 joint_angles;
    float x_hip = position[0] - 42.0f;
    float y_hip = position[1] - 42.0f;
    joint_angles[0] = atan2f(y_hip, x_hip) - M_PI_2;
    if ((joint_angles[0] < MotorJointBounds[0].joint_min)
        || (joint_angles[0]> MotorJointBounds[0].joint_max)){

        return false;
    }

    float x = sqrtf( x_hip*x_hip + y_hip * y_hip);
    float z = position[2];

    float remainder = (x - 132.0f) * x + z * z + 9182.0f;
    float A = -150.0f * x - 30.0f * z + 9900.0f;
    float B = -30.0f * x + 150.0f * z + 1980.0f;
    float R = signf(A) * sqrtf(A * A + B * B);

    if ((R < eps) && (R> -eps))
        return false;

    float C = remainder / R;

    if ((C> 1.0f) || (C < -1.0f))
        return false;

    joint_angles[1] = -atan2f(B/R, A/R) - asinf(C);
    if ((joint_angles[1] < MotorJointBounds[1].joint_min)
        || (joint_angles[1]> MotorJointBounds[1].joint_max)){
        return false;
    }

    float sin_wh = (75.0f* ( 1 - cosf(joint_angles[1])) - 15.0f*sinf(joint_angles[1])) / 32.0f;
    if ((sin_wh >1.0f) || (sin_wh < -1.0f))
        return false;

    joint_angles[2] = asinf(sin_wh);

    if ((joint_angles[2] < MotorJointBounds[2].joint_min)
        || (joint_angles[2]> MotorJointBounds[2].joint_max)){
        return false;
    }

    return joints_to_motors(joint_angles, motor_angles);
}



