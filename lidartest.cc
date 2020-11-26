//
// Simple algorithm for finding doorways
// Written by Eric Gregori
//
//  10.0.0.239 PI
//  ./LidarBot /dev/ttyUSB1 /dev/ttyUSB0
#include <signal.h>
#include "lidarbot.h"

// Personality
#define TOO_CLOSE_ACTUAL  (float)(0.5)
#define LOOKS_INTERESTING_ACTUAL (float)(1.5)

// Used to exit the main loop via ctrl-c signal
static bool running = true;

// Trap exit signals to properly shutdown the hardware
// including stopping the motors.
static void Stop(int signo)   
{  
    printf("Received exit signal\n");
    running = false;
}  

int main(int argc, char **argv)
{
    bool            save = false;

    // Verify command line
    if( argc == 2 )
        save = false;
    else if( argc == 3 )
        save = true;
    else
    {
        printf("\n\nCommand line\n\n");
        printf("%s %s", argv[0], "[lidar /dev/ttyUSB1] <filename>\n\n");
        return -1;
    }

    // Install signal handlers
    signal(SIGINT, Stop);
    signal(SIGTERM, Stop);

    printf("Instantiating LidarBot class\n");
    LidarBot *robot = new LidarBot(NULL, argv[1]);

    printf("Start the loop\n");
    while(running)
    {
        try
        {
            robot->ReadLidarRaw();
        }
        catch(...)
        {
            printf("Error reading lidar\n");
        }
        robot->VisualizeRanges();
    }

    exit(0);
}
