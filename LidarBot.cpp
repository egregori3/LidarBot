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

// Break the 360 degree scan into SECTORS
#define SECTORS     74
#define F_SECTOR    (SECTORS/2)
#define OFFSET      (SECTORS/4)
#define L_SECTOR    (F_SECTOR-OFFSET)
#define R_SECTOR    (F_SECTOR+OFFSET)
#define TOO_CLOSE   (float)(0.1)

enum STATE  {
                STATE_TURN_TO_MAX,
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
static int ReadLidar( CYdLidar *laser, float *max_range, int *direction_of_max_range, float *sectors )
{
    bool        hardError;
    LaserScan   scan;                   // Lidar results
    int         i;

    *direction_of_max_range = -1;
    if(laser->doProcessSimple(scan, hardError))
    {
        int points  = (unsigned int)scan.ranges.size();
        int samples = (points/SECTORS);

        memset((void *)sectors, 0, sizeof(float)*SECTORS);
        for(i=0; i<points; ++i)
        {
            sectors[i/samples] += scan.ranges[i];
            if(i && i%samples == 0)
                sectors[(i/samples)-1] /= samples;
        }
        for(i=0, *max_range = 0.0;  i<SECTORS; ++i)
        {
            if( sectors[i] > *max_range )
            {
                *max_range = sectors[i];
                *direction_of_max_range = i;
            }
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
        for(int k=0; k<(int)(40.0*sectors[i]/(points/SECTORS)); ++k)
            printf("*");
        if( direction_of_max_range == i )
            printf(">");
    }

    return 0;
}
// Ultimately, the goal is to code the application in Python on a laptop 
// and "mount" the robot drivers using gRPC.
// For now, lets just try some simple algorithms in good old C++.
int main(int argc, char **argv)
{
    float       sectors[SECTORS];       // Break down the scans into SECTORS
    float       max_range;              // Furthest distance measured
    int         direction_of_max_range; // Sector containing furthest distance
    STATE       state;

    // Verify command line
    if( argc != 3 )
    {
        printf("\n\nCommand line\n\n");
        printf("%s %s", argv[0], "[lidar /dev/ttyUSB1] [miiboo /dev/ttyUSB0]\n\n");
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

    // Set initial state
    state = STATE_TURN_TO_MAX;

    printf("Start the loop\n");
    while(running)
    {
        miiboo_object->move((unsigned char *)"r");
        if( int points = ReadLidar( laser, &max_range, &direction_of_max_range, sectors ) )
        {
            float left     = sectors[L_SECTOR];
            float right    = sectors[R_SECTOR];
            float forward  = sectors[F_SECTOR];

            switch(state)
            {
                case STATE_TURN_TO_MAX:
                    printf("rotate %f %f %f\n", left, forward, right);
                    miiboo_object->move((unsigned char *)"r");
                    if(direction_of_max_range == F_SECTOR)
                    {
                        miiboo_object->move((unsigned char *)"s");
                        state = STATE_MOVE_FORWARD;
                    }
                    break;
                case STATE_MOVE_FORWARD:
                    // LM(0-9) = 4.5*F+4.5*R    RM(0-9) = 4.5*F+4.5*L
                    int speed[] = {'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o'};
                    unsigned char m[3];

                    printf("forward %f %f %f ", left, forward, right);
                    m[0] = 'm';
                    m[1] = speed[(int)(4.5*forward+4.5*right)];
                    m[2] = speed[(int)(4.5*forward+4.5*left)];
                    printf(" %c %c\n", m[1], m[2]);
                    miiboo_object->move((unsigned char *)"f");
                    if(left < TOO_CLOSE || right < TOO_CLOSE || forward < TOO_CLOSE)
                    {
                        miiboo_object->move((unsigned char *)"s");
                        state = STATE_TURN_TO_MAX;
                    }
                    break;
            }
#if 0
            VisualizeRanges( sectors, points, direction_of_max_range );
#endif
        }
        usleep(25*1000);
    }

    miiboo_object->move((unsigned char *)"s");
    laser->turnOff();
    laser->disconnecting();
    delete miiboo_object;
} // main()

