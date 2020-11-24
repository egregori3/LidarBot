#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <signal.h>
#include <memory>
#include <unistd.h>
#include "CYdLidar.h"
#include "miiboo_driver.h"
#include "miiboo_driver_class.h"


//
// C++ interface to miiboo driver and lidar 
// Written by Eric Gregori
//
//  10.0.0.239 PI
//  ./LidarBot /dev/ttyUSB1 /dev/ttyUSB0

// Break the scan into SECTORS
#define SCAN_START      180
#define SCAN_END        540
#define SCANS           (SCAN_END-SCAN_START)
#define SECTORS         36
#define WIDTH           4
#define L_SECTOR_START  ((SECTORS/2)-WIDTH-(WIDTH/2))
#define L_SECTOR_END    ((SECTORS/2)-(WIDTH/2))
#define R_SECTOR_END    ((SECTORS/2)+WIDTH+(WIDTH/2))
#define R_SECTOR_START  ((SECTORS/2)+(WIDTH/2))
#define F_SECTOR_START  L_SECTOR_END
#define F_SECTOR_END    R_SECTOR_START
#define SCANS_SECTOR    (SCANS/SECTORS)

// Personality
#define TOO_CLOSE_ACTUAL  (float)(0.5)
#define LOOKS_INTERESTING_ACTUAL (float)(1.5)

enum STATE  {
                STATE_STOP,
                STATE_SET_TURN,
                STATE_TURN,
                STATE_MOVE_FORWARD
            };

using namespace std;
using namespace ydlidar;

// Pull in the Lidar driver
CYdLidar laser;

// Used to exit the main loop via ctrl-c signal
static bool running = true;

// Trap exit signals to properly shutdown the hardware
// including stopping the motors.
static void Stop(int signo)   
{  
    printf("Received exit signal\n");
    running = false;
}  

// Read the lidar
static int ReadLidar( CYdLidar *laser, int *direction_of_max_range, float *norm_sectors, float *actual_sectors )
{
    bool        hardError;
    LaserScan   scan;                   // Lidar results
    float       max;
    int         i;

    *direction_of_max_range = -1;

    // Get data from Lidar
    if(laser->doProcessSimple(scan, hardError))
    {
        int points  = (unsigned int)scan.ranges.size();

        printf("points = %d\n", points);
        // Partition scan ranges into sectors with each sector equal to the max range in the sector.
        for(int k=0; k<SECTORS; ++k)
        {
            // sector equal max range in sector
            for(i=0, max=0.0; i<SCANS_SECTOR; ++i)
            {
                float range = scan.ranges[SCAN_START+k*SCANS_SECTOR+i];
                if( range > max)
                    max = range;
            }
            norm_sectors[k] = max;
            actual_sectors[k] = max;
        }

        // Find the sector with the longest distance
        for(i=0, max=0.0; i<SECTORS; ++i)
        {
            if( norm_sectors[i] > max )
            {
                max = norm_sectors[i];
                *direction_of_max_range = i;
            }
        }

        // Normalize ranges to 1.0  (range/max_range)
        if( max > 0.0 )
        {
            for(i=0; i<SECTORS; ++i)
                norm_sectors[i] = norm_sectors[i]/max;
        }

        return(points);
    }

    return -1;
}

// Visualize the data
static int VisualizeRanges( float *sectors, int points, int direction_of_max_range )
{
    for(int i=0; i<SECTORS; ++i)
    {
        printf("\n %0d", i);
        for(int k=0; k<(int)(40.0*sectors[i]); ++k)
        {
            if( i>=F_SECTOR_START && i<=F_SECTOR_END )
                printf("F");
            else if( i>=L_SECTOR_START && i<= L_SECTOR_END )
                printf("L");
            else if( i>=R_SECTOR_START && i<= R_SECTOR_END )
                printf("R");
            else
                printf("*");
        }
        if( direction_of_max_range == i )
            printf(">");
        printf(" - %f",sectors[i]);
    }

    return 0;
}

float average_sectors( int start, int end, float *sectors )
{
    float count = 0.0;
    float avg = 0.0;

    for(int i=start; i<=end; ++i)
    {
        avg += sectors[i];
        if( sectors[i] > 0.0 )
            count += 1.0;
    }

    avg /= count;
    return avg;
}

// Ultimately, the goal is to code the application in Python on a laptop 
// and "mount" the robot drivers using gRPC.
// For now, lets just try some simple algorithms in good old C++.
int main(int argc, char **argv)
{
    float       sectors[SECTORS];           // Break down the scans into SECTORS - nromalized
    float       actual_sectors[SECTORS];    // Break down the scans into SECTORS - actul
    int         direction_of_max_range;     // Sector containing furthest distance
    STATE       state;
    unsigned char turn;
    bool        move = false;

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

    // Configure lidar driver
    printf("\n\nInstantiate lidar driver\n");
    CYdLidar *laser = new CYdLidar();
    laser->setSerialPort(argv[1]);
    laser->setSerialBaudrate(115200);
    laser->setIntensities(0);
    laser->initialize();

    // Instantiate the motor driver
    printf("Instantiating driver\n");
    miiboo_driver *miiboo_object = new miiboo_driver(argv[2]);
    miiboo_object->move((unsigned char *)"s");

    // Set initial state
    if( move )
        state = STATE_SET_TURN;
    else
        state = STATE_STOP;

    printf("Start the loop\n");
    while(running)
    {
        printf("\n");
        if( int points = ReadLidar( laser, &direction_of_max_range, sectors, actual_sectors ) )
        {
            float left = average_sectors( L_SECTOR_START, L_SECTOR_END, sectors );
            float right = average_sectors( R_SECTOR_START, R_SECTOR_END, sectors );
            float forward = average_sectors( F_SECTOR_START, F_SECTOR_END, sectors );
            float left_actual = average_sectors( L_SECTOR_START, L_SECTOR_END, actual_sectors );
            float right_actual = average_sectors( R_SECTOR_START, R_SECTOR_END, actual_sectors );
            float forward_actual = average_sectors( F_SECTOR_START, F_SECTOR_END, actual_sectors );

 #if 1
            printf("%d-%d %d-%d %d-%d\n", L_SECTOR_START, L_SECTOR_END, F_SECTOR_START, F_SECTOR_END, R_SECTOR_START, R_SECTOR_END);
            printf("%f %f %f %d\n", left, forward, right, direction_of_max_range);
            printf("   %f\n", forward_actual);
            switch(state)
            {
                case STATE_STOP:
                    printf("stop\n");
                    miiboo_object->move((unsigned char *)"s");
                    break;
                case STATE_SET_TURN:
                    printf("rotate\n");
                    if(direction_of_max_range < F_SECTOR_START)
                    {
                        turn = 'l';
                        miiboo_object->move((unsigned char *)"l");
                    }
                    else
                    {
                        turn = 'r';
                        miiboo_object->move((unsigned char *)"r");
                    }

                    state = STATE_TURN;
                    break;
                case STATE_TURN:
                    printf("turning\n");
                    miiboo_object->move(&turn);

                    if( forward_actual > LOOKS_INTERESTING_ACTUAL )
                    {
                        miiboo_object->move((unsigned char *)"s");
                        state = STATE_MOVE_FORWARD;
                    }
                    break;
                case STATE_MOVE_FORWARD:
                    // LM(0-9) = 4.5*F+4.5*R    RM(0-9) = 4.5*F+4.5*L
                    int speed[] = {'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'};
                    unsigned char m[3];

                    printf("forward ");
                    m[0] = 'm';
                    m[1] = speed[(int)(7.0*forward+7.0*right)];
                    m[2] = speed[(int)(7.0*forward+7.0*left)];
                    printf(" %c %c\n", m[1], m[2]);
                    miiboo_object->move(m);
                    if(right_actual < TOO_CLOSE_ACTUAL || forward_actual < TOO_CLOSE_ACTUAL || left_actual < TOO_CLOSE_ACTUAL)
                    {
                        miiboo_object->move((unsigned char *)"s");
                        state = STATE_SET_TURN;
                    }
                    break;
            }
#endif
#if 1
            VisualizeRanges( sectors, points, direction_of_max_range );
#endif
        }
    }

    miiboo_object->move((unsigned char *)"s");
    laser->turnOff();
    laser->disconnecting();
    sleep(1);
    delete laser;
    delete miiboo_object;
    sleep(1);
    exit(0);
} // main()

