#include <stdio.h>
#include "minunit.h"
#include "../model.h"

int tests_run = 0;




static char* run_model_tests() {

	return NULL;
}

int main(int argc, char** argv) {

	char* result = run_model_tests();

	if (result != NULL) {
		printf("%s\n", result);

	}
	else {
		printf("Tests Passed.\n");
	}
		
	return result != NULL;
}

