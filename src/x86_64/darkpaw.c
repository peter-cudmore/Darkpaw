//
// Created by Pete on 6/04/2021.
//

#include "../darkpaw.h"

int initialise(struct DarkpawState* state){
    return 0;
};

void shutdown(void){};

void read_sensors(struct DarkpawState* state, float delta_time){};
void compute_next_action(struct DarkpawState* state, float delta_time){};
void update_actuators(struct DarkpawState* state, float delta_time){};
