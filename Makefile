DIR = /home/ubuntu/
RESULT = lidarBot
MIIBOO_LIB = $(DIR)/miiboo_driver/libmiiboo_class.a
MIIBOO_INC = $(DIR)/miiboo_driver/include
LIDAR_LIB  = $(DIR)/lidar/lidar_driver.a
LIDAR_INC  = $(DIR)/lidar/include

SRCS = LidarBot.cpp
OBJS = $(SRCS:.cpp=.o)

.PHONY: clean

all:
	g++ -I$(MIIBOO_INC) -I$(LIDAR_INC) -std=c++11 -Wall -lrt -pthread $(SRCS) $(MIIBOO_LIB) $(LIDAR_LIB) -o LidarBot

clean:
	rm $(RESULT) || true
	rm *.o || true
	rm $(RESULT)_driver.a

