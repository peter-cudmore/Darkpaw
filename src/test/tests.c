#include <stdio.h>
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
				motor_pwm[i] = angle_to_pwm(motor_angles[i] + 0.2f);
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
	return 0;
}


static char* run_model_tests() {
	mu_run_test(test_FK_symmetries);
	mu_run_test(test_inverse_kinematics);
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

