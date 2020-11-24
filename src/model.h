#include <cglm/cglm.h>
#include <stdbool.h>

#ifndef MODEL_H
#define MODEL_H

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
void solve_leg_angles(unsigned motor_pwm[3], LegAngles out_angles);
struct Darkpaw* new_model(void);

void angles_to_leg_position(enum Leg leg, LegAngles angles, vec3 out_array);

bool leg_position_to_angles(enum Leg leg,  vec3 target, LegAngles* out_angles, unsigned current_servos_setpoints[LEG_MOTORS]);


void get_next_motor_position(
		struct Darkpaw *darkpaw, 
		vec2 body_velocity, 
		float body_angular_velocity,
		float height,
		float delta_time,
		unsigned* motor_out[LEGS * LEG_MOTORS]);


int angle_to_pwm(float radians);
float pwm_to_angle(int pwm);
#endif // !MODEL_H




