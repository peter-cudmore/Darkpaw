#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include "model.h"

#include "darkpaw.h"
#define SIGQUIT 3

bool should_quit = false;
#define DT_MS 10L // 0.1s
#define DT 0.01f

void on_quit(int sig)
{
    should_quit = true;
};


int nsleep(long miliseconds)
{
    struct timespec req, rem;

    if (miliseconds > 999)
    {
        req.tv_sec = (int)(miliseconds / 1000);                            /* Must be Non-Negative */
        req.tv_nsec = (miliseconds - ((long)req.tv_sec * 1000)) * 1000000; /* Must be in range of 0 to 999999999 */
    }
    else
    {
        req.tv_sec = 0;                         /* Must be Non-Negative */
        req.tv_nsec = miliseconds * 1000000;    /* Must be in range of 0 to 999999999 */
    }

    return nanosleep(&req, &rem);
}
int main(int arvc, char* argv[]) {

    int exit_code = 0;
    // startup routine
    if ((exit_code = initialise()) != 0) {
	printf("Failed to start\n");
	shutdown();
        return exit_code;
    }

    signal(SIGINT, on_quit);
    signal(SIGQUIT, on_quit);
    /*
    while (!should_quit) {
        step_test(DT);
        nsleep(DT_MS);
    }
    */ 
    repl_loop();
    // each tick
    // read sensors
    // read command
    // estimate position and world
    // update footstep planner
    // generate motor positions
    // continue

    shutdown();
    return 0;
} 
