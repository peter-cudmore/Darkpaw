#include <stdlib.h>
#include <time.h>
#include "../darkpaw.h"
#include "../model.h"


struct Darkpaw *darkpaw = NULL;
unsigned servos[] = { 300, 300, 300 };
vec3 target = { 0.0f , 0.0f, 0.0f };



int initialise(void) {
	darkpaw = new_model();
	return 0;
};

void shutdown(void) {
	if (darkpaw)
		free(darkpaw);
};

void print_readout(struct Darkpaw* darkpaw) {

	float control_velocity = 0.0f, control_angular=0.0f;
	printf("Target:  (%f m/s, %f rad/s)\n", control_velocity, control_angular);
	printf("Feet Positions\n");
	for (int i = 0; i < LEGS; i++) {
		printf("x_%d = (%f, %f, %f), omega = (%f, %f, %f, %f, %f, %f)\n");
	}

	
	

}


void repl_loop(bool * should_quit) {
	printf("Starting Darkpaw Test");
	float t = 0.0f;
	float dt = .01f;
	
};


void step_test(float dt) {
	enum Leg leg = BackLeft;
	LegAngles out_angles;
	vec3 position;
	float t = 0.0f;
	vec3 dir = {0.1 * cosf(0.5 * t) , 0 ,0 };
	
	
	glm_vec3_sub(target, dir, target);

	bool result = leg_position_to_angles(leg, target, &out_angles, servos);

	// angles_to_leg_position(leg, &out_angles, &position);
	
	servos[Angle] = angle_to_pwm(leg, out_angles[TorsoServoArm]);
	servos[Height] = angle_to_pwm(leg, out_angles[BottomServoArm]);
	servos[Radius] = angle_to_pwm(leg, out_angles[TopServoArm]);
 	printf("Front left Positoin : [%f %f %f]\n", position[0], position[1], position[2]);
	printf("             Target : [%f %f %f]\n", target[0], target[1], target[2]);
	printf("			 Servos : [%i, %i, %i]\n", servos[0],servos[1], servos[2]);
	
	t += dt;

};