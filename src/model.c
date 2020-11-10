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
	float s_d = sinf(angles[LegTriangle]);
	float s_y = sinf(angles[LegY]);
	float c_a = cosf(angles[TorsoLeg]);
	float c_d = cosf(angles[LegTriangle]);
	float c_y = cosf(angles[LegY]);

	out_array[0] = -32.0f * s_a * s_d - 15.0f * s_a * s_y + 75.0f * s_a * c_y + 75.0f * s_y * c_a + 32.0f * c_a * c_d + 15.0f * c_a * c_y + 66.0f * c_a + 42.0f;

	if ((leg == BackLeft)|| (leg == BackRight))
		out_array[0] *= -1.0f;
	
	out_array[1] = -(75.0f * s_a * s_y + 32.0f * s_a * c_d + 15.0f * s_a * c_y + 66.0f * s_a + 42.0f);

	if ((leg == FrontRight) || (leg == BackRight))
		out_array[1] *= -1.0f;
	
	out_array[2] = 32.0f * s_d + 15.0f * s_y - 75.0f * c_y;
	
};


void solve_leg_angles(unsigned motor_pwm[3], LegAngles out_angles) {
	float u_b = pwm_to_angle(motor_pwm[Angle]);
	float u_h = pwm_to_angle(motor_pwm[Height]);
	float u_r = pwm_to_angle(motor_pwm[Radius]);

	float cos_coeff_b = 400.0 * sinf(u_b) + 900.0 * cosf(u_b) - 1344.0;
	float sin_coeff_b = 900.0 * sinf(u_b) + 400.0 * cosf(u_b) - 3024.0;
	float remainder_b = 1872.25 - 1050.0 * sinf(u_b);
	// result = solve_implicit_atan(cos_coeff_b, sin_coeff_b, remainder_b)

	float cos_coeff_r = 225.0 * sinf(u_r) - 1170.0 * cosf(u_r) - 987.0;
	float sin_coeff_r = -1170.0 * sinf(u_r) - 225.0 * cosf(u_r) + 2398.5;
	float remainder_r = -960.0 * sinf(u_r) + 195.0 * cosf(u_r) + 2192.5;

	float cos_coeff_h = -650.0 * cos(u_h) - 338.0;
	float sin_coeff_h = -650.0 * sin(u_h) - 1664.0;
	float remainder_h = 800.0 * sin(u_h) + 162.5 * cos(u_h) + 602.5;
	
	out_angles[TorsoServoArm] = u_b;
	out_angles[TorsoLeg] = solve_implicit_atan(cos_coeff_b, sin_coeff_b, remainder_b);
	out_angles[BottomServoArm] = u_h;
	out_angles[LegTriangle] = solve_implicit_atan(cos_coeff_h, sin_coeff_h, remainder_h);
	out_angles[TopServoArm] = u_r;
	out_angles[LegY] = solve_implicit_atan(cos_coeff_r, sin_coeff_r, remainder_r);
	
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
enum Directions inverse_direction[TOTAL_DIR] = {
	DOWN,
	UP,
	RIGHT,
	LEFT,
	BACK,
	FORWARDS
};

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

bool leg_position_to_angles(enum Leg leg, vec3 target, LegAngles* out_angles, unsigned current_servos_setpoints[LEG_MOTORS]){

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
	unsigned steps_left = 200;
	vec3 test_pos, difference;
	enum Directions best_dir = NO_DIR, last_step = NO_DIR;

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

		steps_left--;
	}
	solve_leg_angles(best_motor_values, *out_angles);
	
	return true;
}





void stroke_phase(	vec3 desired_position, 
					enum Leg leg, 
					unsigned motor_now[LEG_MOTORS], 
					unsigned motor_desired[LEG_MOTORS]) {	
	
	
}
