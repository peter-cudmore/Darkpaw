#include <cglm/cglm.h>
#include <stdbool.h>
#include "types.h"

#ifndef MODEL_H
#define MODEL_H

typedef struct 
{
	unsigned min;		
	unsigned max;
	unsigned rest;
	unsigned reference_pwm;
	float reference_angle;
	float gradient;
} LegMotorConfig;

/* Linear Model Parameters
*
* Let:
*	- u_i be the pwm value of the ith motor.
*   - w_i be the angle
*
* Then:
*	- min < u_i < max
*   - w_i = grad * (u_i - rest)
*
*/

// motor bounds
// channel min max
//
//  0 Front Left Body		125 (furthest back)		425 (furthest forwards)			300 (centred)
// 1 Front Left Height		175 (Furthest lowered)		475 (furthest raised)		310 (centred)
// 2 Front Left Radius      100 (Furthest out)			375 (Furthest in)			280 (centred)
//  3 Back Left Body		125 (furthers forwards)	425 (furthest back)				275 (centred)
// 4 Back Left Height	    125 (Furthest Raised)		425 (Furthest lowered)		305
// 5 Rear Left Radius		225 (Furthest in)			500 (furthest out)			315			
//  6 Front right body		175	(furthest forwards	475 (furthest back)				325
// 7 Front Right Height		125 (Furthest Raised)		425 (Furthest Lowered)		300
// 8 Front Right Radius		200 (Furthest in)			450 (Furthest out)			290
//  9 Back right body		175 (furthest back)		475 (furthest forwards)			310
// 10 Back Right Height		175 (Furthest Lowered)		475 (Furthest raised)		300
// 11 Rear right Radius		150 (Furthest out)			375 (Furthest in)			320

extern const LegMotorConfig leg_bounds[12];
extern const int motor_sign[4];
extern vec3 rest_positions[4];

// note:
// COM is _just_ undenearh support when front legs are furthest back, and back legs are at 200/400 respectively
//

enum Leg {
	FrontLeft = 0,
	BackLeft = 1,
	FrontRight = 2,
	BackRight = 3,
	LEGS
};

enum MotorPosition {
	Angle = 0,
	Height = 1,
	Radius = 2,
	LEG_MOTORS
};

enum LegConfigurationAngles {
	TorsoLeg=0,					// What we care about
	TorsoServoArm,				// What we can actuate
	LegTriangle,				// 	
	BottomServoArm,				
	LegY,
	TopServoArm,
	LEG_CONFIGURATION_SIZE
};

enum StrokePhase {
	Planted = 0,
	Power,
	Lifting, 
	Reseting,
	Planting
};

void foot_position_of(enum Leg leg, unsigned motor_pwm[3], vec3 posititon_out);

bool motor_delta_for(
	enum Leg leg,
	vec3 position_delta,
	vec3 current_position,
	unsigned current_motor_pwm[3],
	short motor_delta_out[3]
	);



unsigned get_servo_index(enum Leg leg, enum MotorPosition position);

typedef float LegAngles[LEG_CONFIGURATION_SIZE];

struct LegState {
	float		current_angles[LEG_CONFIGURATION_SIZE];
	unsigned	servos_setpoints[LEG_MOTORS];
	vec3		displacement;						//relative to rest.
	enum Leg	leg;
	enum StrokePhase  phase;

};

struct Darkpaw 
{
    vec3       foot_positons[LEGS];			// in body frame
    bool       is_planted[LEGS];			// true if the leg is planted.
    LegAngles  angles[LEGS];			        // in radians
    unsigned   motor_positions[LEGS][LEG_MOTORS];
    vec3       centre_of_pressure;		        // in body frame
};
void solve_leg_angles(enum Leg leg, unsigned motor_pwm[3], LegAngles out_angles);
struct Darkpaw* new_model(void);

void angles_to_leg_position(enum Leg leg, LegAngles angles, vec3 out_array);

bool leg_position_to_angles(enum Leg leg,  vec3 target, LegAngles* out_angles, unsigned *current_servos_setpoints);


void get_next_motor_position(
		struct Darkpaw *darkpaw, 
		vec2 body_velocity, 
		float body_angular_velocity,
		float height,
		float delta_time,
		unsigned* motor_out[LEGS * LEG_MOTORS]);


unsigned angle_to_pwm(unsigned motor, float radians);
float pwm_to_angle(unsigned motor, unsigned pwm);

void leg_angles_to_pwm(enum Leg leg, LegAngles angles, unsigned *motor_out);


void phase_to_position(enum Leg leg, float phase, vec3 out_position);

struct MotorSequence {
	unsigned current_position;
	unsigned length;
	motor_set *values;
};

struct MotorSequence* allocate_motor_sequence(unsigned int length);

void free_motor_sequence(struct MotorSequence* seq);
/*
bool generate_pick_and_place(enum Leg leg,
	unsigned *current_motor_values,
	vec3 target_position,
	float peak_height,
	float path_duration,
	float delta_time,
	struct MotorSequence* out_sequence);
	*/
#endif // !MODEL_H




