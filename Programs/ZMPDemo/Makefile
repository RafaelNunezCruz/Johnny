###############################################################
#
# Purpose: Makefile for ZMPDemo program
# Author.: Rafael
# Version: 0.1
# License: GPL
#
###############################################################

TARGET = ZMPDemo
OBJS =	main.o 

CXX = g++
INCLUDE_DIRS = -I../../include
CXXFLAGS +=	-O2 -DLINUX -g -Wall -fmessage-length=0 $(INCLUDE_DIRS) `pkg-config opencv --cflags`
LIBS += `pkg-config opencv --libs` -lserial #-lpthread -lrt 



all: Johnny_Framework.a $(TARGET)

Johnny_Framework.a:
	make -C ../../build

$(TARGET): $(OBJS) ../../lib/Johnny_Framework.a
	$(CXX) -o $(TARGET) $(OBJS) ../../lib/Johnny_Framework.a $(LIBS)
	chmod 755 $(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
	rm -f cartesians.txt joints.txt sensors.txt
clear:
	cd ../../build && make clean
