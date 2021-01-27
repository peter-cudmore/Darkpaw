#include <stdio.h>
#include <time.h>

#include "minunit.h"
#include "../model.h"


int tests_run = 0;

void print_vec3(vec3 v) {
	printf("x = (%f, %f, %f)\n", v[0] , v[1], v[2]);
};
void print_angles(LegAngles angles) {
	printf("u = (%f, %f, %f), ", angles[TorsoServoArm], angles[TopServoArm], angles[BottomServoArm]);
	printf("z = (%f, %f, %f)\n", angles[TorsoLeg], angles[LegY], angles[LegTriangle]);
}


static char* test_forward_kinematics_rest(void) {

	LegAngles test_angles;
	unsigned motor_pwm[3];
	float motor_angles[3] = { 0.0f, 0.0f, 0.0f };
	vec3 test_position;
	
	const float epsilon = 0.0001;
	printf("Testing FK at rest: ");
	// Set up base positions and make sure coordinates system is Front-Left-Up
	for (enum Leg leg = 0; leg < 4; leg++) {
		for (int i = 0; i < 3; i++)
			motor_pwm[i] = angle_to_pwm(get_servo_index(leg, i), motor_angles[i]);

		printf("%d", leg);
		fflush(stdout);

		solve_leg_angles(leg, motor_pwm, test_angles);
		angles_to_leg_position(leg, test_angles, test_position);

		glm_vec3_sub(test_position, rest_positions[leg], test_position);
		float test_value = glm_vec3_norm(test_position);
		
		mu_assert(".... FAILED\nTest position is not close to reference value!\n", test_value < epsilon);
	}
	printf(" - Done\n");
	return 0;
}


static char* test_forward_kinematics_directions(){
	// TEST CASES
	// given x_i = f(z_i) and g(z_i, u_i) = 0
	// solve g(z_i', u_i + d_i) and assert that dot(c, f(z_i') - x_i) > 0
	typedef struct {
		enum Leg leg;
		enum MotorPosition motor;
		short motor_delta;
		vec3 c;
	} TestCase;

	TestCase test_cases[] = {
		{FrontLeft,		Angle,		10,		{1.0f, -0.1f, 0.0f}},
		{FrontLeft,		Radius,		-40,	{0.3f, 0.6f, 0.0f}},
		{FrontLeft,		Height,		10,		{0.0f, 0.0f, 1.0f}},
		{FrontRight,	Angle,		10,		{1.0f, 0.1f, 0.0f}},
		{FrontRight,	Radius,		10,		{0.1f, -1.0f, 0.0f}},
		{FrontRight,	Height,		10,		{0.0f, 0.0f, 1.0f}},
		{BackLeft,		Angle,		10,		{1.0f, -0.1f, 0.0f}},
		{BackLeft,		Radius,		10,		{0.1f, 1.0f, 0.0f}},
		{BackLeft,		Height,		10,		{0.0f, 0.0f, 1.0f}},
		{BackRight,		Angle,		10,		{1.0f, 0.1f, 0.0f}},
		{BackRight,		Radius,		10,		{0.1f, -1.0f, 0.0f}},
		{BackRight,		Height,		10,		{0.0f, 0.0f, 1.0f}}
	};

	const int n_cases = 12;
	
	
	unsigned motor_pwm[3];
	vec3 test_position;
	unsigned int motor_idx;
	printf("Testing FK at movement:  ");
	for (int i = 0; i < n_cases; i++) {
		printf("%d", i);
		fflush(stdout);
		LegAngles test_angles = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

		TestCase* test_case = test_cases + i;
		// reset motor vallues
		
		for (int motor = 0; motor < LEG_MOTORS; motor++) {
			motor_idx = get_servo_index(test_case->leg, motor);
			motor_pwm[i] = leg_bounds[motor_idx].rest;
		}
		motor_idx = test_case->motor;
		
		motor_pwm[motor_idx] += leg_bounds[motor_idx].gradient > 0 ? test_case->motor_delta : -test_case->motor_delta;

		solve_leg_angles(test_case->leg, motor_pwm, test_angles);
		angles_to_leg_position(test_case->leg, test_angles, test_position);
		
		float lower_bound = glm_vec3_dot(test_case->c, rest_positions[test_case->leg]);
		float test_value = glm_vec3_dot(test_case->c, test_position);
		float epsilon = 0.01;
		if (test_value > lower_bound + epsilon) {
			printf("\n");
			print_vec3(test_position);
			print_vec3(rest_positions[test_case->leg]);
			print_angles(test_angles);

			mu_assert(" - FAILED!\nFoot moved in wrong direction\n", false );
		}
	}
	printf("Done\n");
	return 0;
}


static char* test_inverse_kinematics(void) {
	// TEST CASE
	// Given an x = f(z) and 0 = g(z, u) and a desired x'
	// Solve for x' = f(z'), g(z', u')
	// with 
	

	LegAngles base_angles = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	LegAngles out_angles;
	unsigned current_motor_pwm[3];
	float motor_angles[3] = { 0.0f, 0.0f, 0.0f };
	vec3 direction = { 2.0f, 0.0f, 0.0f };
	vec3 position =	{ 0.0f, 0.0f, 0.0f };
	vec3 next_pos, difference;
	float norm;
	bool has_solution;

	for (int i = 0; i < 3; i++) { current_motor_pwm[i] = angle_to_pwm(FrontLeft, motor_angles[i]);}

	solve_leg_angles(FrontLeft, current_motor_pwm, base_angles);
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
	
	for (int i = 0; i < 3; i++) { current_motor_pwm[i] = angle_to_pwm(i, motor_angles[i]); }

	solve_leg_angles(FrontLeft, current_motor_pwm, base_angles);
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

static char* test_pick_and_place(void) {

	//	vec3 positions;			->	get current positons
	//	vec3 target_position;	->	positions + (x, 0, 0)
	//	generate a timespan		->	0 to 1s
	//	height	= 2

	vec3 current_position = { rest_positions[0][0], rest_positions[0][1], rest_positions[0][2] };;
	vec3 place_position = { current_position[0] + 2.0f, current_position[1], current_position[2] };
	float height = 1.5f;
	float duration = 3.0f;
	float delta_time = 0.1f;

	struct MotorSequence* out_sequence;

	unsigned current_motor_values[3] = {
		angle_to_pwm(0, 0.0f),
		angle_to_pwm(1, 0.0f),
		angle_to_pwm(2, 0.0f)
	};

	/*
	bool result = generate_pick_and_place(FrontLeft,
		current_motor_values,
		place_position,
		height,
		duration,
		delta_time,
		out_sequence);
	
	mu_assert("Could not find place position: " , result);
	mu_assert("Out Sequence is null", out_sequence != NULL);

	/*
	* 1. test that motor values are valid
	* 2. test that they are 'close enough' 
	* 3. test that the foot follows the desired trajectory dX = (2, 0, M_PI*h*cos(t))dt
	* 4.
	* 
	for (int i = 0; i < out_sequence->length; i++) {

		for (int j = 0; j < 3; j++){

			mu_assert("Motor value is too small", 
				out_sequence->values[3*i + j] > 100);
			mu_assert("Motor value is too big", 
				out_sequence->values[3 * i + j] < 500);

		}
	}
	*/
	return 0;
}


static char* run_model_tests() {
	mu_run_test(test_forward_kinematics_rest);
	mu_run_test(test_forward_kinematics_directions);
	mu_run_test(test_inverse_kinematics);
	mu_run_test(test_ik_bounds);
	mu_run_test(test_pick_and_place);
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

