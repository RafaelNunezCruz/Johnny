###############################################################
#
# Purpose: Makefile for "Johnny Linux Framework"
# Author.: Rafael
# Version: 0.1
# License: GPL
#
###############################################################

CC = g++
AR = ar
ARFLAGS = cr

TARGET = Johnny_Framework.a 

INCLUDE_DIRS = -I../include -I../include/Robotis

CXXFLAGS += -O2 -DLINUX -Wall -shared $(INCLUDE_DIRS) 
 
OBJS =  Robotis/LinuxCM730.o \
		Robotis/CM730.o \
		ActuatorsManipulation.o \
		SensorsManipulation.o \
		OutputFileManipulation.o \
		RobotPose.o \
		WalkingPattern.o \
		InverseKinematics.o \
		Stability.o \
		ArtificialVision.o \
		FilteredSignal.o \
		Johnny.o

$(TARGET): $(OBJS)
	$(AR) $(ARFLAGS) ../lib/$(TARGET) $(OBJS)

clean:
	rm -f $(OBJS)
	cd ../lib && rm -f Johnny_Framework.a
