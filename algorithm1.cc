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
#define FORWARD_GAIN      (float)(16.0)
#define OFFSET_GAIN       (float)(8.0)

enum STATE  {
                STATE_STOP,
                STATE_SET_TURN,
                STATE_TURN_LEFT,
                STATE_TURN_RIGHT,
                STATE_MOVE_FORWARD
            };

STATE Algorithm(STATE, LidarBot *);

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
    printf("\033[2J");
    while(running)
    {
        state = Algorithm(state, robot);
    }

    robot->Move((unsigned char *)"s");
    delete robot; // call desctructor
    exit(0);
}

STATE Algorithm(STATE state, LidarBot *robot)
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
    switch(state)
    {
        case STATE_STOP:
            printf("\nSTATE: STOP\n");
            robot->Move((unsigned char *)"s");
            break;
        case STATE_SET_TURN:
            printf("\nSTATE: LOOKING FOR SOMETHING INTERESTING\n");
            if(robot->furthest.raw < robot->f_sector_start)
                state = STATE_TURN_LEFT;
            else
                state = STATE_TURN_RIGHT;
            break;
        case STATE_TURN_LEFT:
            printf("\nSTATE: TURNING LEFT\n");
            robot->Move((unsigned char *)"l");

            if( forward.raw > LOOKS_INTERESTING_ACTUAL )
            {
                robot->Move((unsigned char *)"s");
                state = STATE_MOVE_FORWARD;
            }
            break;
        case STATE_TURN_RIGHT:
            printf("\nSTATE: TURNING RIGHT\n");
            robot->Move((unsigned char *)"r");

            if( forward.raw > LOOKS_INTERESTING_ACTUAL )
            {
                robot->Move((unsigned char *)"s");
                state = STATE_MOVE_FORWARD;
            }
            break;
        case STATE_MOVE_FORWARD:
            unsigned char m[3];

            printf("\nSTATE: FORWARD ");
            m[0] = 'm';
            m[1] = (unsigned char)('A'+(int)(FORWARD_GAIN*forward.norm+OFFSET_GAIN*left.norm));
            m[2] = (unsigned char)('A'+(int)(FORWARD_GAIN*forward.norm+OFFSET_GAIN*right.norm));
            printf(" %c %c\n", m[1], m[2]);
            robot->Move(m);
            if(right.raw < TOO_CLOSE_ACTUAL || forward.raw < TOO_CLOSE_ACTUAL || left.raw < TOO_CLOSE_ACTUAL)
            {
                robot->Move((unsigned char *)"s");
                state = STATE_SET_TURN;
            }
            break;
    } // switch
    robot->VisualizeRanges();
    return state;
}
