
RESULT = lidarBot
MIIBOO_LIB = /Development/miiboo_controller/libmiiboo_class.a
MIIBOO_INC = /Development/miiboo_controller/include
LIDAR_LIB = /Development/lidar/lidar_driver.a
LIDAR_INC = /Development/lidar/include

SRCS = LidarBot.cpp
OBJS = $(SRCS:.cpp=.o)

.PHONY: clean

all:
	g++ -I$(MIIBOO_INC) -I$(LIDAR_INC) -std=c++11 -lrt -pthread $(SRCS) $(MIIBOO_LIB) $(LIDAR_LIB) -o LidarBot

clean:
	rm $(RESULT) || true
	rm *.o || true
	rm $(RESULT)_driver.a

