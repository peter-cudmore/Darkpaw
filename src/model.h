#include <cglm/cglm.h>
#include <stdbool.h>

#ifndef MODEL_H
#define MODEL_H

#define LEGS 4
#define LEG_MOTORS 3

enum Leg {
	FrontLeft = 0,
	BackLeft = 1,
	FrontRight = 2,
	BackRight = 3
};

enum MotorPosition {
	Angle = 0,
	Height = 1,
	Radius = 2
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



unsigned get_servo_index(enum Leg leg, enum MotorPosition position);

typedef float LegAngles[LEG_CONFIGURATION_SIZE];

struct Darkpaw 
{
    vec3       foot_positons[LEGS];			// in body frame
    bool       is_planted[LEGS];			// true if the leg is planted.
    LegAngles  angles[LEGS];			        // in radians
    unsigned   motor_positions[LEGS][LEG_MOTORS];
    vec3       centre_of_pressure;		        // in body frame
};

struct Darkpaw* new_model(void);

void get_leg_position(enum Leg leg, LegAngles *angles, vec3 *out_array);

void get_next_motor_position(
		struct Darkpaw *darkpaw, 
		vec2 body_velocity, 
		float body_angular_velocity,
		float height,
		float delta_time,
		unsigned* motor_out[LEGS * LEG_MOTORS]);


#endif // !MODEL_H




