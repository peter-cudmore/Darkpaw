#include "minunit.h"
#include "../kinematics.h"
#include <math.h>
int tests_run = 0;

#define EPSILON 1e-7
#define vec3_set(v, x, y, z) v[0] = (float)x; v[1] = (float)y; v[2] = (float)z

#define equal_eps(a, b) fabsf(a-b) < EPSILON 


static char* basic_operations() {

	// Model looks like in the xy plane
	//
	//    + - e 
	//    |
	//    o
	//
	//

	KineChain chain;
	
	vec3 e_x = { 1, 0, 0 };
	vec3 e_y = { 0, 1, 0 };
	vec3 e_z = { 0, 0, 1 };
	int rot_1, rot_2, link_1, link_2;
	init_chain(&chain);
	
	mu_assert("Failed to create a revolute joint", (rot_1 = add_new_revolute_joint(&chain, e_z)) == 0 );
	mu_assert("Failed to create a fixed joint", (rot_2 = add_fixed_rotation(&chain, e_z, -M_PI_2)) == 1);
	mu_assert("Failed to create links",	(2 == (link_1 = add_link(&chain, e_y))) 
									 && (3 == (link_2 = add_link(&chain, e_x))));
	
	int end_effector[4] = { rot_1, link_1, link_2, rot_2};

	KineIndexList effector = { 
		.indicies = end_effector,
		.size = 4,
		.max = 4
	};
	
	define_end_effector(&chain, effector);
	
	float angles[1] = { 0.0f };
	vec3 test, reference, temp;
	vec3 zero = { 0,0,0 };
	
	printf("Testing basic operations...");
	vec3_set(reference, 1, 1, 0);
	mu_assert("Reference transform failed", transform_vector(&chain, angles, zero, test));
	if (!glm_vec3_eqv_eps(test, reference)) {
		fprintf(stderr, "Result: (%f, %f, %f)\n", test[0], test[1], test[2]);
		mu_assert("Reference transform is incorrect",false );
	}


	// shfting everything by e_y should put the end effector at (1, 2, 0);
	vec3_set(reference, 1, 2, 0);
	mu_assert("y-axis shift failed", transform_vector(&chain, angles, e_y, test));
	if (!glm_vec3_eqv_eps(test, reference)) {
		fprintf(stderr, "\nResult:");
		glm_vec3_print(test, stderr);
		mu_assert("y-axis shift is wrong", false);
	}

	// rotating by pi/2 should put the end effector at (-1, 1, 0)
	angles[0] = M_PI_2;	
	vec3_set(reference, -1, 1, 0);
	mu_assert("Controlled rotating failed", transform_vector(&chain, angles, zero, test));
	mu_assert("Controlled rotating is invalid", glm_vec3_eqv_eps(test, reference));

	// Solve IK problem
	// Target is (sqrt(2), 0, 0) requiring an angle of pi/4
	vec3_set(test, M_SQRT2, 0, 0);
	mu_assert("Could not solve inverse_kinematics", find_angles(&chain, test, angles));
	mu_assert("IK reesult is wrong", equal_eps(M_PI_4, angles[0]));

	printf("Done\n");

	return 0;
}


static char* run_all_tests() {
	mu_run_test(basic_operations);
    return 0;
}


int main() {    
    char* result = run_all_tests();
	
	if (result != NULL) {
		printf("%s\n", result);
	}
	else {
		printf("Tests Passed.\n");
	}

	return result != NULL;
}