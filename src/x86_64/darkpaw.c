#include "../darkpaw.h"
#include "../model.h"


struct Darkpaw *darkpaw = NULL;

int initialise(void) {
	darkpaw = new_model();
	printf("New Darkpaw with C.O.P [%f,%f, %f]\n",
		darkpaw->centre_of_pressure[0], darkpaw->centre_of_pressure[1], darkpaw->centre_of_pressure[2]);
	return 0;
};

void shutdown(void) {
	if (darkpaw)
		free(darkpaw);
};

void step_test(float dt) {
	
	//get_next_motor_position(darkpaw,);



};