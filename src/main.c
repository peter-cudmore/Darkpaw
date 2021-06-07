#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>

#include "darkpaw.h"
#define SIGQUIT 3

bool should_quit = false;
#define DT_MS 10L // 0.01s

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
    struct DarkpawState state;
    // startup routine
    if ((exit_code = initialise(&state)) != 0) {
	    printf("Failed to start!\n");
        goto shutdown;
    }

    signal(SIGINT, on_quit);
    signal(SIGQUIT, on_quit);
    float dt = DT_MS / 1000.0f;
    while (!should_quit){
        // read from sensors

        read_sensors(&state, dt);
        // compute next action

        compute_next_action(&state, dt);

        update_actuators(&state, dt);

        nsleep(DT_MS);
    }

shutdown:
    shutdown();
    return 0;
} 
