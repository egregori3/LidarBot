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


enum STATE  {
                STATE_STOP,
                STATE_SET_TURN,
                STATE_TURN_LEFT,
                STATE_TURN_RIGHT,
                STATE_MOVE_FORWARD
            };

int Algorithm(STATE, LidarBot *);

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
    STATE           state;
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

    // Set initial state
    if( move )
        state = STATE_SET_TURN;
    else
        state = STATE_STOP;

    printf("Start the loop\n");
    while(running)
    {
        if(Algorithm(state, robot)<0)
            break;
    }

    robot->Move((unsigned char *)"s");
    exit(0);
}

int Algorithm(STATE state, LidarBot *robot)
{
    SECTOR left, right, forward;

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
    printf("\n\n\nnormalized: %f %f %f\n", left.norm, forward.norm, right.norm);
    printf("raw: %f %f %f\n", left.raw, forward.raw, right.raw);
    printf("furthest: %f %d\n", robot->max_range, robot->direction_of_max_range);
    robot->VisualizeRanges();
    printf("%d-%d %d-%d %d-%d\n", robot->l_sector_start, robot->l_sector_end,
                                  robot->f_sector_start, robot->f_sector_end, 
                                  robot->r_sector_start, robot->r_sector_end);
    switch(state)
    {
        case STATE_STOP:
            printf("stop\n");
            robot->Move((unsigned char *)"s");
            break;
        case STATE_SET_TURN:
            printf("rotate\n");
            if(robot->direction_of_max_range < robot->f_sector_start)
                state = STATE_TURN_LEFT;
            else
                state = STATE_TURN_RIGHT;
            break;
        case STATE_TURN_LEFT:
            printf("Turning left\n");
            robot->Move((unsigned char *)"l");

            if( forward.raw > LOOKS_INTERESTING_ACTUAL )
            {
                robot->Move((unsigned char *)"s");
                state = STATE_MOVE_FORWARD;
            }
            break;
        case STATE_TURN_RIGHT:
            printf("Turning right\n");
            robot->Move((unsigned char *)"r");

            if( forward.raw > LOOKS_INTERESTING_ACTUAL )
            {
                robot->Move((unsigned char *)"s");
                state = STATE_MOVE_FORWARD;
            }
            break;
        case STATE_MOVE_FORWARD:
            // LM(0-9) = 4.5*F+4.5*R    RM(0-9) = 4.5*F+4.5*L
            int speed[] = {'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'};
            unsigned char m[3];

            printf("forward ");
            m[0] = 'm';
            m[1] = speed[(int)(7.0*forward.norm+7.0*right.norm)];
            m[2] = speed[(int)(7.0*forward.norm+7.0*left.norm)];
            printf(" %c %c\n", m[1], m[2]);
            robot->Move(m);
            if(right.raw < TOO_CLOSE_ACTUAL || forward.raw < TOO_CLOSE_ACTUAL || left.raw < TOO_CLOSE_ACTUAL)
            {
                robot->Move((unsigned char *)"s");
                state = STATE_SET_TURN;
            }
            break;
    } // switch
}
