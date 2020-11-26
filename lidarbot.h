//
// C++ interface to miiboo driver and lidar 
// Written by Eric Gregori
//
#ifndef LIDARBOT_H_
#define LIDARBOT_H_
#include "CYdLidar.h"
#include "miiboo_driver.h"
#include "miiboo_driver_class.h"

using namespace ydlidar;
using namespace std;

typedef struct 
{
    bool  max;
    char  type;
    int   point;
    float raw;
    float norm;
}   SECTOR;

class LidarBot
{
    private:
        miiboo_driver  *miiboo_object;
        CYdLidar       *laser;
        LaserScan      scan;         // Lidar results
        vector<SECTOR> points;
        bool           hardError;

        float AverageSectors(int, int, char);

    public:
        int l_sector_start, r_sector_start;
        int l_sector_end, r_sector_end;
        int f_sector_start, f_sector_end;
        SECTOR furthest, closest;

        LidarBot(char *motor_port, char *lidar_port);
        void GetSectors(SECTOR *left, SECTOR *forward, SECTOR *right);
        void ReadLidarRaw(void);
        void VisualizeRanges(void);
        void Move(unsigned char *cmd)
        {
            if(miiboo_object != NULL)
                miiboo_object->move(cmd);
        }
        ~LidarBot(void);
};

#endif
