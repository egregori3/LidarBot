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

typedef struct 
{
    float actual;
    float norm;
}   SECTOR;

class LidarBot
{
    private:
        miiboo_driver *miiboo_object;
        CYdLidar      *laser;
        LaserScan     scan;         // Lidar results
        SECTOR        *sectors;
        bool          hardError;

        float AverageSectors(int, int, char);

    public:
        SECTOR left, right, forward;
        int l_sector_start, r_sector_start;
        int l_sector_end, r_sector_end;
        int f_sector_start, f_sector_end;
        int direction_of_max_range; // Sector containing furthest distance

        LidarBot(char *motor_port, char *lidar_port);
        int ReadLidar(void);
        int VisualizeRanges(void);
        void Move(unsigned char *cmd)
        {
            miiboo_object->move(cmd);
        }
        ~LidarBot(void);
};

#endif
