#include "../darkpaw.h"
#include "../model.h"


struct Darkpaw *darkpaw = NULL;
unsigned servos[] = { 300, 300, 300 };
vec3 target = { 0.0f , 0.0f, 0.0f };



int initialise(void) {
	darkpaw = new_model();
	printf("New Darkpaw with C.O.P [%f, %f, %f]\n",
		darkpaw->centre_of_pressure[0], darkpaw->centre_of_pressure[1], darkpaw->centre_of_pressure[2]);
	return 0;
};

void shutdown(void) {
	if (darkpaw)
		free(darkpaw);
};


float t = 0.0f;


void step_test(float dt) {
	enum Leg leg = BackLeft;
	LegAngles out_angles;
	vec3 position;
	vec3 dir = {0.1 * cosf(0.5 * t) , 0 ,0 };

	
	glm_vec3_sub(target, dir, target);

	bool result = leg_position_to_angles(leg, target, &out_angles, servos);

	// angles_to_leg_position(leg, &out_angles, &position);
	
	servos[Angle] = angle_to_pwm(out_angles[TorsoServoArm]);
	servos[Height] = angle_to_pwm(out_angles[BottomServoArm]);
	servos[Radius] = angle_to_pwm(out_angles[TopServoArm]);
 	printf("Front left Positoin : [%f %f %f]\n", position[0], position[1], position[2]);
	printf("             Target : [%f %f %f]\n", target[0], target[1], target[2]);
	printf("			 Servos : [%i, %i, %i]\n", servos[0],servos[1], servos[2]);
	
	t += dt;

};