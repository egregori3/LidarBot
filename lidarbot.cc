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
#define CENTER          (((SCAN_END-SCAN_START)/2)+SCAN_START)
#define WIDTH           60  // degrees*2
#define R_SECTOR_START  (CENTER-WIDTH-(WIDTH/2))
#define R_SECTOR_END    (CENTER-(WIDTH/2))
#define L_SECTOR_START  (CENTER+(WIDTH/2))
#define L_SECTOR_END    (CENTER+WIDTH+(WIDTH/2))
#define F_SECTOR_START  R_SECTOR_END
#define F_SECTOR_END    L_SECTOR_START

using namespace std;

void LidarBot::ReadLidarRaw(void)
{
    if(laser->doProcessSimple(scan, hardError))
    {
        int     size  = (int)scan.ranges.size();
        int     max_point = 0;
        SECTOR  tmp;

        // max
        float max = 0.0;
        points.clear();
        for(int i=0; i<size; ++i)
        {
            float range = scan.ranges[i];
            if (range > max)
            {
                max = range;
                max_point = i;
            }

            tmp.max = false;
            tmp.type = '*';
            tmp.point = i;
            tmp.raw = range;
            tmp.norm = range;

            if(i >= L_SECTOR_START && i < L_SECTOR_END)
                tmp.type = 'L';
            else if(i > R_SECTOR_START && i <= R_SECTOR_END)
                tmp.type = 'R';
            else if(i >= F_SECTOR_START && i <= F_SECTOR_END)
                tmp.type = 'F';
            else
                tmp.type = '*';

            points.push_back(tmp);
        }

        points[max_point].max = true;
        direction_of_max_range = max_point;
        max_range = max;

        if(max == 0.0)
        {
            throw(runtime_error("max 0"));
        }

        // Normalize
        for(int i=0; i<(int)points.size(); ++i)
        {
            points[i].norm /= max;
        }
    }
}

// Format points into l,f,r sectors
void LidarBot::GetSectors(SECTOR *left, SECTOR *forward, SECTOR *right)
{
    float leftc, rightc, forwardc;
    int pointc = (int)points.size();

    // Get data from Lidar
    leftc  = 0.0;
    rightc = 0.0;
    forwardc = 0.0;
    left->norm = 0.0;
    right->norm = 0.0;
    forward->norm = 0.0;
    left->raw = 0.0;
    right->raw = 0.0;
    forward->raw = 0.0;

    if(pointc > 0)
    {
        for(int i=0; i<(int)points.size(); ++i)
        {
            if(points[i].raw > 0.0)
            {
                if(points[i].type == 'L')
                {
                    left->raw += points[i].raw;
                    left->norm += points[i].norm;
                    leftc += 1.0;
                }
                if(points[i].type == 'F')
                {
                    forward->raw += points[i].raw;
                    forward->norm += points[i].norm;
                    forwardc += 1.0;
                }
                if(points[i].type == 'R')
                {
                    right->raw += points[i].raw;
                    right->norm += points[i].norm;
                    rightc += 1.0;
                }
            }
        }

        left->raw /= leftc;
        left->norm /= leftc;
        right->raw /= rightc;
        right->norm /= rightc;
        forward->raw /= forwardc;
        forward->norm /= forwardc;
    }
}

// Visualize the data in a 40 x 40 space
void LidarBot::VisualizeRanges(void)
{
    int points_per_column = points.size()/40;
    int k, i, id;

    printf("\n%d-%d %d-%d %d-%d\n", l_sector_start, l_sector_end,
                                  f_sector_start, f_sector_end, 
                                  r_sector_start, r_sector_end);

    for(i=0; i<((int)points.size()-points_per_column); )
    {
        float max = 0.0;
        for(k=0, id=0; k<points_per_column; ++k)
        {
            if(points[i].norm > max)
            {
                max = points[i].norm;
                id = i;
            }
            ++i;
        }

        printf("\n");
        for(k=0; k<(int)(80.0*max); ++k)
        {
            printf("%c", points[id].type);
        }
        if(points[i].max == true)
            printf("!");
        printf(" -  %f", max);
    }
}

LidarBot::LidarBot(char *motor_port, char *lidar_port)
{
    printf("LidarBot Constructor\n");
    l_sector_start = L_SECTOR_START;
    r_sector_start = R_SECTOR_START;
    f_sector_start = F_SECTOR_START;
    l_sector_end = L_SECTOR_END;
    r_sector_end = R_SECTOR_END;
    f_sector_end = F_SECTOR_END;

    // Configure lidar driver
    printf("\n\nInstantiate lidar driver\n");
    laser = new CYdLidar();

    // Instantiates lidar
    laser->setSerialPort(lidar_port);
    laser->setSerialBaudrate(115200);
    laser->setIntensities(0);
    laser->initialize();

    if(motor_port != NULL)
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

