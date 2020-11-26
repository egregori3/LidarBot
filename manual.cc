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


void Algorithm(LidarBot *);

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
    LidarBot *robot;
    // Install signal handlers
    signal(SIGINT, Stop);
    signal(SIGTERM, Stop);

    // Verify command line
    if( argc == 2 )
        robot = new LidarBot(NULL,argv[1]);
    else if( argc == 3 )
        robot = new LidarBot(argv[2], argv[1]);
    else
    {
        printf("\n\nCommand line\n\n");
        printf("%s %s", argv[0], "[lidar /dev/ttyUSB0] <motor /dev/ttyUSB1>\n\n");
        return -1;
    }

    robot->Move((unsigned char *)"s");

    printf("Start the loop\n");
    while(running)
    {
        Algorithm(robot);
    }

    robot->Move((unsigned char *)"s");
    delete robot;
    exit(0);
}

void Algorithm(LidarBot *robot)
{
      SECTOR left, right, forward;
 
      robot->Move((unsigned char *)"r");
      printf("\n");
      try
      {
          robot->ReadLidarRaw();
      }
      catch(...)
      {  
          printf("Error reading lidar\n");
      }
      robot->GetSectors(&left, &forward, &right);
      printf("\033[0;0H");
      for(int i=0; i<45; ++i)
          printf("%90c\n", ' ');
      printf("\033[0;0H");
      printf("\n\n\n");
      printf("%d-%d %d-%d %d-%d\n", robot->l_sector_start, robot->l_sector_end,
                                    robot->f_sector_start, robot->f_sector_end, 
                                   robot->r_sector_start, robot->r_sector_end);
     printf("normalized: %8f %8f %8f\n", left.norm, forward.norm, right.norm);
     printf("raw:        %8f %8f %8f\n", left.raw, forward.raw, right.raw);
     printf("furthest:   %8f %d\n", robot->furthest.raw, robot->furthest.point);
     printf("closest:    %8f %d\n", robot->closest.raw, robot->closest.point);
     robot->VisualizeRanges();
}
