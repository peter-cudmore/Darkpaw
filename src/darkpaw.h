#ifndef DARKPAW_H
#include "types.h"


int initialise(struct DarkpawState* state);
void shutdown(void);

void read_sensors(struct DarkpawState*, float delta_time);
void compute_next_action(struct DarkpawState*, float delta_time);
void update_actuators(struct DarkpawState*, float delta_time);


#define DARKPAW_H
#endif // !DARKPAW_H
