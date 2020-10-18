#include "model.h"
#include <math.h>

#define MOTOR_ZERO 300
#define eps 0.0001f

float abs_f(float x) {
	return x > 0 ? x : -x;
}
float sign_f(float x) {
	return x < 0 ? -1.0f : 1.0f;
}

int angle_to_pwm(float angle) {
	return (int) round((1 + angle / M_2_PI) * MOTOR_ZERO);
}

float pwm_to_angle(int pwm) {
	return M_PI * (((float)pwm) / MOTOR_ZERO - 1.0f);
}

unsigned get_servo_index(enum Leg leg, enum MotorPosition position) {
	return ( 3 * (unsigned)leg) + ((unsigned) position);
}

float solve_implicit_atan(float cos_coeff, float sin_coeff, float remainder) 
{
	float abs_sin_coeff = fabsf(sin_coeff);
	float abs_cos_coeff = fabsf(cos_coeff);
	float phi, c;

	if (abs_cos_coeff < eps * abs_sin_coeff) {
			// function is approximately  A*sin(x) + remainder = 0
		return asinf(-remainder / sin_coeff);
	}
	else if (abs_sin_coeff < eps * abs_cos_coeff) {
		// function is approex B* cos(x) + remainder = 0
		return acosf(-remainder / cos_coeff);
	}
	c = sign_f(cos_coeff) * sqrtf(cos_coeff * cos_coeff + sin_coeff * sin_coeff);
	phi = atan2f(-sin_coeff, cos_coeff);
	// now have c*cos(x + phi) = - remainder
	return acosf(-remainder / c) - phi;
}

void get_leg_position(enum Leg leg, LegAngles *angles, vec3 *out_array) {
	
	float s_a = sinf(*angles[TorsoLeg]);
	float s_d = sinf(*angles[LegTriangle]);
	float s_y = sinf(*angles[LegY]);
	float c_a = cosf(*angles[TorsoLeg]);
	float c_d = cosf(*angles[LegTriangle]);
	float c_y = cosf(*angles[LegY]);

	*out_array[0] = -32.0 * s_a * s_d - 15.0 * s_a * s_y + 75.0 * s_a * c_y + 75.0 * s_y * c_a + 32.0 * c_a * c_d + 15.0 * c_a * c_y + 66.0 * c_a + 42.0;

	if (leg == BackLeft || leg == BackRight) 
		*out_array[0] *= -1.0;
	
	*out_array[1] = 75.0 * s_a * s_y + 32.0 * s_a * c_d + 15.0 * s_a * c_y + 66.0 * s_a + 42.0;
	
	if (leg == FrontRight || leg == FrontLeft)
		*out_array[1] *= -1.0;
	
	*out_array[2] = 32.0 * s_d + 15.0 * s_y - 75.0 * c_y;

};

void solve_leg_angles(int motor_pwm[3], LegAngles *out_angles) {
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

	*out_angles[TorsoServoArm] = u_b;
	*out_angles[TorsoLeg] = solve_implicit_atan(cos_coeff_b, sin_coeff_b, remainder_b);
	*out_angles[BottomServoArm] = u_h;
	*out_angles[LegTriangle] = solve_implicit_atan(cos_coeff_h, sin_coeff_h, remainder_h);
	*out_angles[TopServoArm] = u_r;
	*out_angles[LegY] = solve_implicit_atan(cos_coeff_r, sin_coeff_r, remainder_r);
}

void update_state_from_motors(struct Darkpaw* darkpaw) {
	
	vec3 running_sum;
	unsigned num_planted = 0;

	glm_vec3_zero(darkpaw->centre_of_pressure);
	glm_vec3_zero(running_sum);
	
	for (int i = 0; i < LEGS; i++) {
		solve_leg_angles(darkpaw->motor_positions[i], &(darkpaw->angles[i]));
		get_leg_position(i, &(darkpaw->angles[i]), &(darkpaw->foot_positons[i]));
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
	short* motor_out[LEGS * LEG_MOTORS]){

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

void fill_neighbours(enum Leg leg, 
					 int motor_now[3],
					 vec3 desired_position,
					 SearchElement elements[2][2][2]) {
	LegAngles temp_angles;
	vec3 this_position;
	
	for (int i = -1; i < 2; i +=2) {
		for (int j = -1; j < 2; j += 2) {
			for (int k = -1; k < 2; k+= 2) {
				elements[1 + i][1 + j][1 + k].motor_values[0] = motor_now[0] + i;
				elements[1 + i][1 + j][1 + k].motor_values[1] = motor_now[1] + j;
				elements[1 + i][1 + j][1 + k].motor_values[2] = motor_now[2] + k;
				solve_leg_angles(elements[1 + i][1 + j][1 + k].motor_values, &temp_angles);
				get_leg_position(leg, &temp_angles, &this_position);
				elements[1 + i][1 + j][1 + k].distance = glm_vec3_distance2(this_position, desired_position);
			}
		}
	}
}


void stroke_phase(	vec3 desired_position, 
					enum Leg leg, 
					int motor_now[LEG_MOTORS], 
					int motor_desired[LEG_MOTORS]) {
	
	
	vec3 temp_position;
	LegAngles temp_angles;
	SearchElement best_element;
	SearchElement neighbours[2][2][2];
	bool found = false;
	unsigned max_steps = 100, current_step=0;

	
	for (unsigned i = 0; i < LEG_MOTORS; i++) {
		best_element.motor_values[i] = motor_now[i];
	}

	solve_leg_angles(best_element.motor_values, &temp_angles);
	get_leg_position(leg, &temp_angles, &temp_position);
	best_element.distance = glm_vec3_distance2(temp_position, desired_position);
	// for each surroudning element
	// compute the distance
	// set the 

	while ((found == false) && (max_steps > current_step)){
		
		fill_neighbours(leg, best_element.motor_values, desired_position, neighbours);

		found = true;
		for (int i = -1; i < 2; i += 2) {
			for (int j = -1; j < 2; j += 2) {
				for (int k = -1; k < 2; k += 2) {
					if (neighbours[i][j][k].distance < best_element.distance) {
						for (unsigned l = 0; l < LEG_MOTORS; l++) {
							best_element.motor_values[l] = neighbours[i][j][k].motor_values[l];
						}
						best_element.distance = neighbours[i][j][k].distance;
						found = false;
					}
				}
			} 
		} 
		current_step++;
	} 
	for (unsigned i = 0; i < LEG_MOTORS; i++) {
		motor_desired[i] = best_element.motor_values[i];
	}
}