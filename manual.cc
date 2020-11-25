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


int Algorithm(LidarBot *);

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
    bool            move = false;

    // Verify command line
    if( argc == 3 )
        move = false;
    else if( argc == 4 )
        move = true;
    else
    {
        printf("\n\nCommand line\n\n");
        printf("%s %s", argv[0], "[lidar /dev/ttyUSB1] [miiboo /dev/ttyUSB0] enable\n\n");
        return -1;
    }

    // Install signal handlers
    signal(SIGINT, Stop);
    signal(SIGTERM, Stop);

    LidarBot *robot = new LidarBot(argv[1], argv[2]);
    robot->Move((unsigned char *)"s");

    printf("Start the loop\n");
    while(running)
    {
        if(Algorithm(robot)<0)
            break;
    }

    robot->Move((unsigned char *)"s");
    exit(0);
}

int Algorithm(LidarBot *robot)
{
        printf("\n");
        if( int points = robot->ReadLidar() )
        {
            printf("%d-%d %d-%d %d-%d\n", robot->l_sector_start, robot->l_sector_end,
                                          robot->f_sector_start, robot->f_sector_end, 
                                          robot->r_sector_start, robot->r_sector_end);
            printf("\n%f %f %f %d\n", robot->left.norm, robot->forward.norm, robot->right.norm, robot->direction_of_max_range);
            printf("%f %f %f\n", robot->left.actual, robot->forward.actual, robot->right.actual);
            robot->Move((unsigned char *)"r");
            robot->VisualizeRanges();
        }
}
