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
#define F_SECTOR_END    R_SECTOR_END
#define TOO_CLOSE       (float)(0.1)
#define SCANS_SECTOR    (SCANS/SECTORS)


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
    float       max;
    int         i;

    *direction_of_max_range = -1;

    // Get data from Lidar
    if(laser->doProcessSimple(scan, hardError))
    {
        int points  = (unsigned int)scan.ranges.size();

        printf("points = %d\n", points);
        for(int k=0; k<SECTORS; ++k)
        {
            max = 0.0;
            for(i=0; i<SCANS_SECTOR; ++i)
            {
                // Scale ranges from 0.0 to 1.0
                float range = scan.ranges[SCAN_START+k*SCANS_SECTOR+i]/10.0;
                if( range > max)
                    max = range;
            }
            sectors[k] = max;
        }

        // Find max
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
        for(int k=0; k<(int)(40.0*sectors[i]); ++k)
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

    printf("Start the loop\n");
    while(running)
    {
        if( int points = ReadLidar( laser, &max_range, &direction_of_max_range, sectors ) )
        {
            float left;
            float right;
            float forward;

            left = 0.0;
            for(int i=L_SECTOR_START; i<=L_SECTOR_END; ++i)
                left += sectors[i];
            right = 0.0;
            for(int i=R_SECTOR_START; i<=R_SECTOR_END; ++i)
                right += sectors[i];
            forward = 0.0;
            for(int i=F_SECTOR_START; i<=F_SECTOR_END; ++i)
                forward += sectors[i];

            forward /= WIDTH;
            right /= WIDTH;
            left /= WIDTH;

            miiboo_object->move((unsigned char *)"r");
            printf("%f %f %f %d %f ", left, forward, right, direction_of_max_range, max_range);
            VisualizeRanges( sectors, points, direction_of_max_range );
        }
        usleep(25*1000);
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

