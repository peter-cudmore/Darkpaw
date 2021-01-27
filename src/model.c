#include "model.h"
#include <math.h>
#include <assert.h>

#define MOTOR_ZERO 300
#define eps 0.0001f

/*
const LegMotorConfig leg_bounds[12] = {
	{125, 425, 300, 425, M_PI_4, (425.0 - 300.0) / M_PI_4},
	{175, 475, 310, 110, -M_PI_2, 200.0f / M_PI_2 },
	{100, 375, 280, 480, M_PI_2, 200.0f / M_PI_2},	// 480 -> flat pointin out
	{125, 425, 275, 390, M_PI_4, 125.0f/M_PI_4  },	// 390 -> 45 deg back
	{125, 425, 300, 500, M_PI_2, 200.0f / M_PI_2},	// 500 -> arm flat poiting ot
	{225, 500, 315, 115, -M_PI_2, 200.0f / M_PI_2 },	// 125 -> arm flat pointing out
	{175, 475, 325, 410, M_PI_4, 125.0f/M_PI_4},	// 410 -> arm point 45 deg back
	{125, 425, 300, 500, M_PI_2, 200.0f/M_PI_2},	// 500 -> arm pointing straight out
	{200, 450, 290, 90, -M_PI_2, 200.0f/M_PI_2},	// 90 -> arm poinint straight out
	{175, 475, 310, 200, -M_PI_4, 125.0f/M_PI_4 },	// 45 deg back
	{175, 475, 300, 100, -M_PI_2, 200.0f/M_PI_2 },	// straight out
	{150, 375, 320, 500, M_PI_2, 200.0f/M_PI_2}	// straight out
};
*/
const LegMotorConfig leg_bounds[12] = {
	{125, 425, 300, 425, M_PI_4, (425.0 - 300.0) / M_PI_4},
	{175, 475, 300, 100, -M_PI_2, 200.0f / M_PI_2 },
	{100, 375, 300, 500, M_PI_2, 200.0f / M_PI_2},	// 480 -> flat pointin out
	{125, 425, 300, 390, M_PI_4, 125.0f / M_PI_4  },	// 390 -> 45 deg back
	{125, 425, 300, 500, M_PI_2, 200.0f / M_PI_2},	// 500 -> arm flat poiting ot
	{225, 500, 300, 115, -M_PI_2, 200.0f / M_PI_2 },	// 125 -> arm flat pointing out
	{175, 475, 300, 410, M_PI_4, 125.0f / M_PI_4},	// 410 -> arm point 45 deg back
	{125, 425, 300, 500, M_PI_2, 200.0f / M_PI_2},	// 500 -> arm pointing straight out
	{200, 450, 300, 90, -M_PI_2, 200.0f / M_PI_2},	// 90 -> arm poinint straight out
	{175, 475, 300, 200, -M_PI_4, 125.0f / M_PI_4 },	// 45 deg back
	{175, 475, 300, 100, -M_PI_2, 200.0f / M_PI_2 },	// straight out
	{150, 375, 300, 500, M_PI_2, 200.0f / M_PI_2}	// straight out
};


float abs_f(float x) {
	return x > 0 ? x : -x;
}
float sign_f(float x) {
	return x < 0 ? -1.0f : 1.0f;
}

unsigned angle_to_pwm(unsigned motor, float radians) {
	const LegMotorConfig *motor_config = &leg_bounds[motor];
	
	if (radians < -M_PI) { radians += M_2_PI; }
	else if (radians > M_PI) { radians -= M_2_PI; }

	unsigned pwm = motor_config->rest + (unsigned)round(motor_config->gradient * radians);
	
	return pwm;
}

float pwm_to_angle(unsigned motor, unsigned pwm) {
	float out = ((float)pwm - (float)leg_bounds[motor].rest) / leg_bounds[motor].gradient;

	return  out;
}

unsigned get_servo_index(enum Leg leg, enum MotorPosition position) {
	return ( 3 * (unsigned)leg) + ((unsigned) position);
}

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


void solve_leg_angles(enum Leg leg, unsigned motor_pwm[3], LegAngles out_angles) {
	float u_b = pwm_to_angle(leg, motor_pwm[Angle]);
	float u_h = pwm_to_angle(leg, motor_pwm[Height]);
	float u_r = pwm_to_angle(leg, motor_pwm[Radius]);

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
		solve_leg_angles(i, darkpaw->motor_positions[i], darkpaw->angles[i]);
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
/*
typedef struct SearchElement {
	float distance;
	int motor_values[LEG_MOTORS];
} SearchElement;

// #define LEG_MOTORS_CUBED LEG_MOTORS * LEG_MOTORS * LEG_MOTORS
*/

enum Directions {NO_DIR = -1, UP = 0, DOWN, LEFT, RIGHT, FORWARDS, BACK, TOTAL_DIR};
enum Directions inverse_direction[TOTAL_DIR] = {DOWN, UP, RIGHT, LEFT, BACK, FORWARDS};

void add_u3(unsigned a[3], int b[3], unsigned out[3]) {
	for (int i = 0; i < 3; i++) {
		out[i] = a[i] + b[i];
	}
}

bool are_motors_valid(motor_set values) {
	for (int i = 0; i < LEG_MOTORS; i++) {
		if ((values[i] > 450) || (values[i] < 150)) {
			return false;
		}
	}
	return true;
}

bool leg_position_to_angles(enum Leg leg, vec3 target, LegAngles* out_angles, unsigned *current_servos_setpoints) {

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
	
	solve_leg_angles(leg, best_motor_values, test_angles);
	
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
				solve_leg_angles(leg, test_motor_values, test_angles);
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

	solve_leg_angles(leg, best_motor_values, *out_angles);
	
	return (!at_boundary);
}


vec3 rest_positions[4] = {
	{65.603020f, 45.364006f,-2.931194f},
	{-65.603020f, 45.364006f, -2.931194f},
	{65.603020f, -45.364006f,-2.931194f},
	{-65.603020f, -45.364006f, -2.931194f} 
};


void phase_to_position(enum Leg leg, float phase, vec3 out_position) {

	if (phase < M_PI) {
		out_position[0] = rest_positions[leg][0] + 4.0f * (phase / M_PI - 0.5f);
		out_position[1] = rest_positions[leg][1];
		out_position[2] = rest_positions[leg][2];
	}
	else {
		out_position[0] = rest_positions[leg][0] - 4.0f * (phase / M_PI - 1.5f);
		out_position[1] = rest_positions[leg][1];
		out_position[2] = rest_positions[leg][2] - 2.0f * sinf(phase);
	}
}

void leg_angles_to_pwm(enum Leg leg, LegAngles angles, unsigned *motor_out) {
	motor_out[Angle] = angle_to_pwm(get_servo_index(leg, Angle), angles[TorsoLeg]);
	motor_out[Height] = angle_to_pwm(get_servo_index(leg, Height), angles[LegTriangle]);
	motor_out[Radius] = angle_to_pwm(get_servo_index(leg, Radius), angles[LegY]);
};




struct MotorSequence* allocate_motor_sequence(unsigned int length) {

	// unsigned* memory = malloc((2 + 3 * length) * sizeof(unsigned));

	// struct MotorSequence* out = (struct MotorSequence*)memory;
	struct MotorSequence* out = malloc(sizeof(struct MotorSequence));
	out->current_position = 0;
	out->length = length;
	out->values = malloc(length * sizeof(motor_set));
	return out;
}

void free_motor_sequence(struct MotorSequence* seq) {
	if (seq) {
		if (seq->values) {
			free(seq->values);
		}
		free(seq);
	}
}
/*
void print_vec3(vec3 vector) {
	printf("(%f, %f, %f)\n", vector[0], vector[1], vector[2]);
}*/

bool
generate_pick_and_place(enum Leg leg,
	motor_set current_motor_values,
	vec3 target_position,
	float peak_height,
	float path_duration,
	float delta_time,
	struct MotorSequence* out_sequence) {

	unsigned int steps = (unsigned)roundf(path_duration / delta_time);
	vec3 position;
	vec3 current_position;
	float proportion;
	float height = 0;
	LegAngles angles;
	unsigned *temp_motor;

	solve_leg_angles(leg, current_motor_values, angles);
	angles_to_leg_position(leg, angles, current_position);

	out_sequence = allocate_motor_sequence(steps);
	
	for (int i = 0; i < 3; i++) { 
		out_sequence->values[0][i] = current_motor_values[i]; 
	}
	printf("%u %u %u\n", out_sequence->values[0][0], out_sequence->values[0][1], out_sequence->values[0][2]);
	temp_motor = &(out_sequence->values[0][0]);
	for (int i = 1; i < steps; i++) {
		proportion = (float)i / (steps - 1);
		glm_vec3_lerp(current_position, target_position, proportion, position);
		height = peak_height * sinf(M_PI * proportion);
		position[2] += height;
		
		printf("Motors: (%u %u %u)",
			temp_motor[0],
			temp_motor[1],
			temp_motor[2]);
		if (leg_position_to_angles(leg, position, &angles, temp_motor) == true) {
			temp_motor = &(out_sequence->values[i][0]);
			leg_angles_to_pwm(leg, angles, temp_motor);
			printf("-> (%u %u %u) \n",
				temp_motor[0],
				temp_motor[1],
				temp_motor[2]);
			printf("Leg angles: (%f, %f, %f, %f,%f, %f)\n",
				angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]);
		}
		else {
			free_motor_sequence(out_sequence);
			out_sequence = NULL;
			return false;
		}

	}
	
	return true;

}