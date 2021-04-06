//
// Created by Pete on 6/04/2021.
//

#include "minunit.h"
#include <stdio.h>
int tests_run = 0;


static char* all_tests(){

    return 0;
}


int main(int argc, char **argv) {
    char *result = all_tests();
    if (result != 0) {
        printf("%s\n", result);
    }
    return result != 0;
}