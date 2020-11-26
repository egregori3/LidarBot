# Written by Eric Gregori

DIR        = ../
APP1       = algorithm1
APP2       = manual
MIIBOO_LIB = $(DIR)/miiboo_driver/libmiiboo_class.a
MIIBOO_INC = $(DIR)/miiboo_driver/include
LIDAR_LIB  = $(DIR)/lidar/lidar_driver.a
LIDAR_INC  = $(DIR)/lidar/include
LIBS       = $(MIIBOO_LIB) $(LIDAR_LIB)
INC        = -I$(LIDAR_INC) -I$(MIIBOO_INC) -I./

SRCS = lidarbot.cc
OBJS = $(SRCS:.cpp=.o)

CFLAGS = -Wall -lrt -pthread

.PHONY: clean

all: $(APP1) $(APP2) 

$(APP1):
	g++ $(INC) -std=c++11 $(CFLAGS) $(SRCS) $(APP1).cc $(LIBS) -o $(APP1)

$(APP2):
	g++ $(INC) -std=c++11 $(CFLAGS) $(SRCS) $(APP2).cc $(LIBS) -o $(APP2)


clean:
	rm $(APP1) || true
	rm $(APP2) || true

