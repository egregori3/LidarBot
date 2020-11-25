//
// C++ interface to miiboo driver and lidar 
// Written by Eric Gregori
//
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <memory>
#include <unistd.h>
#include "lidarbot.h"

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

using namespace std;

// Read the lidar
int LidarBot::ReadLidar(void)
{
    float       max;
    int         i;

    direction_of_max_range = -1;

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
            sectors[k].norm   = max;
            sectors[k].actual = max;
        }

        // Find the sector with the longest distance
        for(i=0, max=0.0; i<SECTORS; ++i)
        {
            if( sectors[i].norm > max )
            {
                max = sectors[i].norm;
                direction_of_max_range = i;
            }
        }

        // Normalize ranges to 1.0  (range/max_range)
        if( max > 0.0 )
        {
            for(i=0; i<SECTORS; ++i)
                sectors[i].norm = sectors[i].norm/max;
        }

        left.norm = AverageSectors(L_SECTOR_START, L_SECTOR_END, 'n' );
        right.norm = AverageSectors(R_SECTOR_START, R_SECTOR_END, 'n' );
        forward.norm = AverageSectors(F_SECTOR_START, F_SECTOR_END, 'n' );
        left.actual = AverageSectors(L_SECTOR_START, L_SECTOR_END, 'a' );
        right.actual = AverageSectors(R_SECTOR_START, R_SECTOR_END, 'a' );
        forward.actual = AverageSectors(F_SECTOR_START, F_SECTOR_END, 'a' );

        return(points);
    }

    return -1;
}

// Visualize the data
int LidarBot::VisualizeRanges(void)
{
    for(int i=0; i<SECTORS; ++i)
    {
        printf("\n %0d", i);
        for(int k=0; k<(int)(40.0*sectors[i].norm); ++k)
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
        printf(" - %f",sectors[i].norm);
    }

    return 0;
}

float LidarBot::AverageSectors(int start, int end, char type)
{
    float count = 0.0;
    float avg = 0.0;

    for(int i=start; i<=end; ++i)
    {
        if( type == 'n' )
        {
            avg += sectors[i].norm;
            if( sectors[i].norm > 0.0 )
                count += 1.0;
        }
        else
        {
            avg += sectors[i].actual;
            if( sectors[i].actual > 0.0 )
                count += 1.0;
        }
    }

    avg /= count;
    return avg;
}

LidarBot::LidarBot(char *motor_port, char *lidar_port)
{
    l_sector_start = L_SECTOR_START;
    r_sector_start = R_SECTOR_START;
    f_sector_start = F_SECTOR_START;
    l_sector_end = L_SECTOR_END;
    r_sector_end = R_SECTOR_END;
    f_sector_end = F_SECTOR_END;
    // Configure lidar driver
    printf("\n\nInstantiate lidar driver\n");
    CYdLidar *laser = new CYdLidar();
    sectors = new SECTOR[SECTORS];
    laser->setSerialPort(lidar_port);
    laser->setSerialBaudrate(115200);
    laser->setIntensities(0);
    laser->initialize();

    if(lidar_port == NULL)
    {
        // Instantiate the motor driver
        printf("Instantiating driver\n");
        miiboo_driver *miiboo_object = new miiboo_driver(motor_port);
        miiboo_object->move((unsigned char *)"s");
    }
    else
        miiboo_driver *miiboo_object = NULL;
}

LidarBot::~LidarBot()
{
    if(miiboo_object != NULL)
        miiboo_object->move((unsigned char *)"s");
    laser->turnOff();
    laser->disconnecting();
    sleep(1);
    delete laser;
    if(miiboo_object != NULL)
        delete miiboo_object;
    sleep(1);
}

