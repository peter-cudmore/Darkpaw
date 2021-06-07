//
// Created by Pete on 6/04/2021.
//
#include <stdio.h>
#include <cglm/cglm.h>
#include "minunit.h"
#include "../types.h"
#include "../kinematics.h"

int tests_run = 0;

#define MOTOR_ZERO 300
vec3 x_rest = {57.144985f, 154.152649f, -70.629044f};

bool is_near(vec3 x, vec3 y){
    vec3 error;
    const float tolerance = 1.0f; //1 mm
    glm_vec3_sub(x, y, error);

    return (glm_vec3_norm(error) < tolerance);
}

void print_vec3(vec3 v){
    printf("(%f, %f, %f)\n",v[0], v[1],v[2]);
}

static char* test_forwards_and_inverse_kinematics(void){
    vec3 motors= {0.0f, 0.0f, 0.0f};

    vec3 x = {0,0,0};

    mu_assert("Failed to find angles", forward_leg_kinematics(motors, x));

    mu_assert("Position should be non-zero", is_near(x, x_rest));

    motors[0]= 0.1f;
    motors[1] = -0.05f;
    motors[2] = 0.01f;

    vec3 zero = {0.0f,0.0f,0.0f};
    mu_assert("Failed to solve inverse kinematics", inverse_leg_kinematics(x_rest, motors));
    mu_assert("Zeros solution is not zero", is_near(motors, zero));

    vec3 x_test_2 = {57.14681243f, 154.118296f, -68.710770f};
    vec3 result_2 = {-0.1342837f, 0.00975966f, 0.1930489f };
    mu_assert("Failed to solve inverse kinematics #2", inverse_leg_kinematics(x_test_2, motors));

    mu_assert("Solution #2 is innaccurate", is_near(motors, result_2 ));

    vec3 x_test_3 = {57.14681243f, 154.118296f, -65.710770f};
    vec3 result_3  = {-0.1342837f,  0.01904501f, 0.28476986f};
    mu_assert("Failed to solve inverse kinematics #3",inverse_leg_kinematics(x_test_3, motors));
    mu_assert("Solution #3 is innaccurate", is_near(motors, result_3 ));

    return NULL;
}


static char* all_tests(){
    mu_run_test(test_forwards_and_inverse_kinematics);
    return NULL;
}


int main(int argc, char **argv) {
    char *result = all_tests();

    if (result != NULL) {
        printf("%s\n", result);
    } else {
        printf("Success\n");
    }

    printf("%d tests run.\n ", tests_run);
    return (result != 0);
}