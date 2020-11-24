#include <stdio.h>
#include <time.h>

#include "minunit.h"
#include "../model.h"

int tests_run = 0;

void print_vec3(vec3 v) {
	printf("%f, %f, %f\n", v[0] , v[1], v[2]);
};
void print_angles(LegAngles angles) {
	printf("%f, %f, %f \n", angles[TorsoServoArm], angles[TopServoArm], angles[BottomServoArm]);
	printf("%f, %f, %f \n", angles[TorsoLeg], angles[LegY], angles[LegTriangle]);
}


static char* test_FK_symmetries(void) {

	// start at motor zero

	LegAngles test_angles = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	unsigned motor_pwm[3];
	float motor_angles[3] = { 0.0f, 0.0f, 0.0f };
	vec3 base_positions[4], test_position;
	float test_value;
	
	// Set up base positions and make sure coordinates system is Front-Left-Up
	for (enum Leg leg = 0; leg < 4; leg++) {

		for (int i = 0; i < 3; i++) {
			motor_pwm[i] = angle_to_pwm(motor_angles[i]);
			mu_assert("Failed to zero motor\n", motor_pwm[i] == 300);
		}

		solve_leg_angles(motor_pwm, test_angles);
		angles_to_leg_position(leg, test_angles, base_positions[leg]);
		mu_assert("Z should be below COM.", base_positions[leg][2] < 0);

		if ((leg == FrontLeft) || (leg == FrontRight)) {
			mu_assert("Front X should be positive ", base_positions[leg][0] > 0);
		}
		else {
			mu_assert("Rear X should be negative ", base_positions[leg][0] < 0);
		}

		if ((leg == FrontLeft) || (leg == BackLeft)) {
			mu_assert("Left Y should be positive", base_positions[leg][1] > 0);
		}
		else {
			mu_assert("Right Y should be negative", base_positions[leg][1] < 0);
		}
	}

	for (enum Leg leg = 0; leg < 4; leg++) {
		for (enum MotorPosition i = 0; i < 3; i++) {
			if (i == Angle) {
				if ((leg == FrontLeft) || (leg == FrontRight)) { motor_pwm[i] = angle_to_pwm(motor_angles[i] + 0.2f); }
				else { motor_pwm[i] = angle_to_pwm(motor_angles[i] - 0.2f); }
			}
			else motor_pwm[i] = angle_to_pwm(0.0f);

		}
		solve_leg_angles(motor_pwm, test_angles);
		angles_to_leg_position(leg, test_angles, test_position);
		mu_assert("Should be shifted forwards", test_position[0] > base_positions[leg][0]);

		for (enum MotorPosition i = 0; i < 3; i++) {
			if (i == Radius) {
				motor_pwm[i] = angle_to_pwm(motor_angles[i] - 0.2f);
			}
			else motor_pwm[i] = angle_to_pwm(0.0f);

		}
		solve_leg_angles(motor_pwm, test_angles);
		angles_to_leg_position(leg, test_angles, test_position);
		if (base_positions[leg][1] > 0) {
			test_value = test_position[1] - base_positions[leg][1];
		}
		else {
			test_value = base_positions[leg][1] - test_position[1];
		}
		mu_assert("Should be shifted out", test_value > 0);
	}

	return 0;
}


static char* test_inverse_kinematics(void) {

	LegAngles base_angles = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	LegAngles out_angles;
	unsigned current_motor_pwm[3];
	float motor_angles[3] = { 0.0f, 0.0f, 0.0f };
	vec3 direction = { 2.0f, 0.0f, 0.0f };
	vec3 position =	{ 0.0f, 0.0f, 0.0f };
	vec3 next_pos, difference;
	float norm;
	bool has_solution;

	for (int i = 0; i < 3; i++) { current_motor_pwm[i] = angle_to_pwm(motor_angles[i]);}

	solve_leg_angles(current_motor_pwm, base_angles);
	angles_to_leg_position(FrontLeft, base_angles, position);

	glm_vec3_sub(position, direction, next_pos);

	has_solution = leg_position_to_angles(FrontLeft, next_pos, &out_angles, current_motor_pwm);

	mu_assert("Cold not solve IK: ", has_solution == true);

	
	glm_vec3_sub(position, next_pos, difference);
	
	norm = glm_vec3_norm(difference);
	mu_assert("Solution is not close to desired state", (direction[0] - 0.1f < norm) && (norm < direction[0] + 0.1f));
	glm_vec3_normalize(difference);
	mu_assert("Solution is in the wrong direction", 0.9f < glm_vec3_dot(difference, direction));


	return 0;
}


static char* test_ik_bounds(void) {
	LegAngles base_angles = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	LegAngles out_angles;
	unsigned current_motor_pwm[3];
	float motor_angles[3] = { 0.0f, 0.0f, 0.0f };
	vec3 direction = { 10.0f, 0.0f, 0.0f };
	vec3 position = { 0.0f, 0.0f, 0.0f };
	vec3 next_pos, difference;
	float norm;
	bool has_solution;
	struct timespec begin, end;
	
	for (int i = 0; i < 3; i++) { current_motor_pwm[i] = angle_to_pwm(motor_angles[i]); }

	solve_leg_angles(current_motor_pwm, base_angles);
	angles_to_leg_position(FrontLeft, base_angles, position);

	glm_vec3_sub(position, direction, next_pos);
	
	clock_gettime(CLOCK_MONOTONIC_RAW, &begin);
	has_solution = leg_position_to_angles(FrontLeft, next_pos, &out_angles, current_motor_pwm);
	clock_gettime(CLOCK_MONOTONIC_RAW, &end);
	
	mu_assert("IK is supposed to fail.", has_solution == false);
	printf("Elapsed time: %li ns", end.tv_nsec - begin.tv_nsec);


	glm_vec3_sub(position, next_pos, difference);
	glm_vec3_normalize(difference);
	mu_assert("Solution is in the wrong direction", 0.9f < glm_vec3_dot(difference, direction));


	return 0;

}

static char* run_model_tests() {
	mu_run_test(test_FK_symmetries);
	mu_run_test(test_inverse_kinematics);
	mu_run_test(test_ik_bounds);
	return NULL;
}

int main(int argc, char** argv) {

	char* result = run_model_tests();

	if (result != NULL) {
		printf("%s\n", result);
	}
	else {
		printf("Tests Passed.\n");
	}
		
	return result != NULL;
}

