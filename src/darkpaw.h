#include "model.h"

#ifndef DARKPAW_H

int initialise(void);
void shutdown(void);
void step_test(float delta_time);
void repl_loop(bool* should_quit);

#define DARKPAW_H
#endif // !DARKPAW_H
