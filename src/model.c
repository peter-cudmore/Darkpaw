#include "model.h"
#include <math.h>
#include <assert.h>

#define MOTOR_ZERO 300
#define eps 0.0001f

float abs_f(float x) {
	return x > 0 ? x : -x;
}
float sign_f(float x) {
	return x < 0 ? -1.0f : 1.0f;
}

int angle_to_pwm(float radians) {
	int pwm = MOTOR_ZERO + round(MOTOR_ZERO * radians / M_PI_2);
	return pwm;
}

float pwm_to_angle(int pwm) {
	return  (((float)pwm) / MOTOR_ZERO - 1.0f) * M_PI_2;
}

unsigned get_servo_index(enum Leg leg, enum MotorPosition position) {
	return ( 3 * (unsigned)leg) + ((unsigned) position);
}

const LegMotorConfig leg_bounds[12] = {
	{125, 425, 300, 425, M_PI_4},
	{175, 475, 310, 110, -M_PI_2},
	{100, 375, 280, 0, 0.0f},
	{125, 425, 275, 0, 0.0f},
	{125, 425, 305, 0, 0.0f},
	{225, 500, 315, 0, 0.0f},
	{175, 475, 325, 0, 0.0f},
	{125, 425, 300, 0, 0.0f},
	{200, 450, 290, 0, 0.0f},
	{175, 475, 310, 0, 0.0f},
	{175, 475, 300, 0, 0.0f},
	{150, 375, 320, 0, 0.0f}
};

const int motor_sign[4] = {
	1,
	-1,
	-1,
	1
};

float solve_implicit_atan(float cos_coeff, float sin_coeff, float remainder) 
{
	float abs_sin_coeff = fabsf(sin_coeff);
	float abs_cos_coeff = fabsf(cos_coeff);
	float phi, c, result; 

	if (abs_cos_coeff < eps * abs_sin_coeff) {
			// function is approximately  A*sin(x) + remainder = 0
		result = asinf(-remainder / sin_coeff);
	}
	else if (abs_sin_coeff < eps * abs_cos_coeff) {
		// function is approex B* cos(x) + remainder = 0
		result =  acosf(-remainder / cos_coeff);
	}
	else
	{
		c = sign_f(cos_coeff) * sqrtf(cos_coeff * cos_coeff + sin_coeff * sin_coeff);
		phi = atan2f(-sin_coeff, cos_coeff);
		// now have c*cos(x + phi) = - remainder
		result = acosf(-remainder / c) - phi;
		if (result > M_PI_2) {result -= M_PI;}
	}

	return result;
}


void angles_to_leg_position(enum Leg leg, LegAngles angles, vec3 out_array) {
	

	float s_a = sinf(angles[TorsoLeg]);
	float s_h = sinf(angles[LegTriangle]);
	float s_r = sinf(angles[LegY]);
	float c_a = cosf(angles[TorsoLeg]);
	float c_h = cosf(angles[LegTriangle]);
	float c_r = cosf(angles[LegY]);

	float s_0 = sinf(angles[TorsoLeg] - angles[LegY]);
	float s_1 = sinf(angles[TorsoLeg] + angles[LegY]);
	float s_2 = sinf(angles[TorsoLeg] - angles[LegTriangle]);
	float s_3 = sinf(angles[TorsoLeg] + angles[LegTriangle]);

	
	float c_0 = cosf(angles[TorsoLeg] - angles[LegY]);
	float c_1 = cosf(angles[TorsoLeg] + angles[LegY]);
	float c_2 = cosf(angles[TorsoLeg] - angles[LegTriangle]);
	float c_3 = cosf(angles[TorsoLeg] + angles[LegTriangle]);
		
	float px = -37.5 * c_0 + 37.5 * c_1 + 75.0 * c_r - 7.5 * s_0 - 7.5 * s_1 - 16.0 * s_2 - 16.0 * s_3 - 66.0 * s_a - 32.0 * s_h - 15.0 * s_r + 42.0;
	float py = 15.0 * c_1 + 32.0 * c_3 + 66.0 * c_a + 75.0 * s_1 + 42.0;
	float pz = -75.0 * c_r + 32.0 * s_h + 15.0 * s_r;

	if ((leg == BackLeft) || (leg == BackRight)) {
		out_array[0] = -px;
	}
	else {
		out_array[0] = px;
	}
	
	out_array[1] = py;
	if ((leg == FrontRight) || (leg == BackRight))
		out_array[1] *= -1.0f;
	
	out_array[2] = pz;
	
};


void solve_leg_angles(unsigned motor_pwm[3], LegAngles out_angles) {
	float u_b = pwm_to_angle(motor_pwm[Angle]);
	float u_h = pwm_to_angle(motor_pwm[Height]);
	float u_r = pwm_to_angle(motor_pwm[Radius]);

	float bodyangle_cos_coeff = -400.0 * sinf(u_b) - 900.0 * cosf(u_b) - 1344.0;
	float bodyangle_sin_coeff = -900.0 * sinf(u_b) + 400.0 * cosf(u_b) - 3024.0;
	float bodyangle_remainder = 1050.0 * sinf(u_b) + 1872.25;

	float radial_angle_cos_coeff = -225.0 * sinf(u_r) - 1170.0 * cosf(u_r) - 987.0;
	float radial_angle_sin_coeff = -1170.0 * sinf(u_r) + 225.0 * cosf(u_r) - 2398.5;
	float radial_angle_remainder = 960.0 * sinf(u_r) + 195.0 * cosf(u_r) + 2192.5;

	float height_angle_cos_coeff = -650.0 * cosf(u_h) - 338.0;
	float height_angle_sin_coeff = 1664.0 - 650.0 * sinf(u_h);
	float height_angle_remainder = -800.0 * sinf(u_h) + 162.5 * cosf(u_h) + 602.5;
	
	out_angles[TorsoServoArm] = u_b;
	out_angles[TorsoLeg] = solve_implicit_atan(bodyangle_cos_coeff, bodyangle_sin_coeff, bodyangle_remainder);
	out_angles[BottomServoArm] = u_h;
	out_angles[LegTriangle] = solve_implicit_atan(height_angle_cos_coeff, height_angle_sin_coeff, height_angle_remainder);
	out_angles[TopServoArm] = u_r;
	out_angles[LegY] = solve_implicit_atan(radial_angle_cos_coeff, radial_angle_sin_coeff, radial_angle_remainder);

}


void update_state_from_motors(struct Darkpaw* darkpaw) {
	
	vec3 running_sum;
	unsigned num_planted = 0;

	glm_vec3_zero(darkpaw->centre_of_pressure);
	glm_vec3_zero(running_sum);
	
	for (int i = 0; i < LEGS; i++) {
		solve_leg_angles(darkpaw->motor_positions[i], darkpaw->angles[i]);
		angles_to_leg_position(i, darkpaw->angles[i], darkpaw->foot_positons[i]);
		if (darkpaw->is_planted[i]) {
			glm_vec3_add(darkpaw->foot_positons[i], running_sum, running_sum);
			num_planted++;
		}
	}
	glm_vec3_muladds(running_sum, 1.0f / ((float)num_planted), darkpaw->centre_of_pressure);
};


struct Darkpaw* new_model(void) {

	struct Darkpaw* darkpaw = malloc(sizeof(struct Darkpaw));
	vec3 zero = { 0.0f, 0.0f, 0.0f };

	if (!darkpaw)
		return NULL;
	
	glm_vec3_zero(darkpaw->centre_of_pressure);

	for (int i = 0; i < LEGS; i++) {
		for (int j =0; j< LEG_MOTORS; j++)
			darkpaw->motor_positions[i][j] = 0;

		darkpaw->is_planted[i] = true;
	}
	update_state_from_motors(darkpaw);
	return darkpaw;
}

void get_next_motor_position(
	struct Darkpaw* darkpaw,
	vec2 body_velocity,
	float body_angular_velocity,
	float height,
	float delta_time,
	unsigned* motor_out[LEGS * LEG_MOTORS]){

	// servo resolution is 150->450 (so 300 samples) 

	// for each planted leg
	// get current position x + 
	// set desired position to be x - v * dt  - dt * (transformed angle)
	// solve local graph search to get next position
	
	// for the lifted leg
	// move it to the reset position.



};

// void lift_phase()
// void reset_phase()
// void stroke_phase()
// void place_phase()

typedef struct SearchElement {
	float distance;
	int motor_values[LEG_MOTORS];
} SearchElement;

// #define LEG_MOTORS_CUBED LEG_MOTORS * LEG_MOTORS * LEG_MOTORS


enum Directions {NO_DIR = -1, UP = 0, DOWN, LEFT, RIGHT, FORWARDS, BACK, TOTAL_DIR};
enum Directions inverse_direction[TOTAL_DIR] = {DOWN, UP, RIGHT, LEFT, BACK, FORWARDS};

void add_u3(unsigned a[3], int b[3], unsigned out[3]) {
	for (int i = 0; i < 3; i++) {
		out[i] = a[i] + b[i];
	}
}

bool are_motors_valid(unsigned values[LEG_MOTORS]) {
	for (int i = 0; i < LEG_MOTORS; i++) {
		if ((values[i] > 450) || (values[i] < 150)) {
			return false;
		}
	}
	return true;
}

bool leg_position_to_angles(enum Leg leg, vec3 target, LegAngles* out_angles, unsigned current_servos_setpoints[LEG_MOTORS]) {

	unsigned best_motor_values[LEG_MOTORS] = { MOTOR_ZERO, MOTOR_ZERO, MOTOR_ZERO };
	int motor_steps[TOTAL_DIR][LEG_MOTORS] = {
		{0, 1, 0},
		{0, -1, 0},
		{0, 0, 1},
		{0, 0, -1},
		{1, 0, 0},
		{-1, 0, 0}
	};
	unsigned zero[3] = { 0,0,0 };
	unsigned test_motor_values[LEG_MOTORS];
	LegAngles test_angles;
	float test_distance, best_distance;
	unsigned steps_left = 1000;
	vec3 test_pos, difference;
	enum Directions best_dir = NO_DIR, last_step = NO_DIR;
	bool at_boundary = false;

	if (current_servos_setpoints != NULL) {
		add_u3(current_servos_setpoints, zero, best_motor_values);
	}
	
	solve_leg_angles(best_motor_values, test_angles);
	
	angles_to_leg_position(leg, test_angles, test_pos);
	glm_vec3_sub(test_pos, target, difference);
	best_distance = glm_vec3_norm(difference);
	
	while (steps_left > 0) {
		for (int dir = 0; dir < TOTAL_DIR; dir++) {
			if (dir != last_step) {
				add_u3(best_motor_values, (motor_steps[dir]), test_motor_values);
				if (!are_motors_valid(test_motor_values)) {
					at_boundary = true;
					continue;
				}
				solve_leg_angles(test_motor_values, test_angles);
				angles_to_leg_position(leg, test_angles, test_pos);
				glm_vec3_sub(test_pos, target, difference);
				test_distance = glm_vec3_norm(difference);
				if (test_distance < best_distance) {
					best_dir = dir;
					best_distance = test_distance;
				}
			}
		}
		
		if (best_dir == NO_DIR) {
			break;
		}

		last_step = inverse_direction[best_dir];
		add_u3(best_motor_values, (motor_steps[best_dir]), best_motor_values);
		best_dir = NO_DIR;
		at_boundary = false;
		steps_left--;
	}

	solve_leg_angles(best_motor_values, *out_angles);
	
	return (!at_boundary);
}




#define MAX_EXTENSION
#define MIN_EXTENSION 


void phase_to_position(enum Leg leg, float phase, vec3 out_position) {


}