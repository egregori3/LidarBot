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


#define SECTORS 74

using namespace std;
using namespace ydlidar;


CYdLidar laser;
static bool running = true;

static void Stop(int signo)   
{  
    printf("Received exit signal\n");
    running = false;
}  


// Testing the driver
int main(int argc, char **argv)
{
    bool        hardError;
    LaserScan   scan;
    float       sector[SECTORS];
    float       furthest;
    int         direction;

    // Verify command line
    if( argc != 3 )
    {
        printf("\n\nCommand line\n\n");
        printf("%s %s", argv[0], "[lidar /dev/ttyUSBx] [miiboo /dev/ttyUSBx]\n\n");
        return -1;
    }

    signal(SIGINT, Stop);
    signal(SIGTERM, Stop);

    laser.setSerialPort(argv[1]);
    laser.setSerialBaudrate(115200);
    laser.setIntensities(0);

    laser.initialize();

    printf("\n\nInstantiating driver\n");
    miiboo_driver *miiboo_object = new miiboo_driver(argv[2]);

    while(running)
    {
        miiboo_object->move((unsigned char *)"r");
        if(laser.doProcessSimple(scan, hardError))
        {
            int points = (unsigned int)scan.ranges.size();

            printf("\n\n\n");
            for(int i=0; i<SECTORS; ++i)
                sector[i] = 0.0;
            for(int i=0; i<points; ++i)
                sector[i/(points/SECTORS)] += scan.ranges[i];
            for(int i=0, furthest = 0.0;  i<SECTORS; ++i)
            {
                if( sector[i] > furthest )
                {
                    furthest = sector[i];
                    direction = i;
                }
            }
            for(int i=0; i<SECTORS; ++i)
            {
                printf("\n %0d", i);
                for(int k=0; k<(int)(40.0*sector[i]/(points/SECTORS)); ++k)
                    printf("*");
                if( direction == i )
                    printf(">");
            }
            if( direction == (38))
                running = false;
        }
        usleep(25*1000);
    }

    miiboo_object->move((unsigned char *)"s");
    laser.turnOff();
    laser.disconnecting();
    delete miiboo_object;
} // main()

