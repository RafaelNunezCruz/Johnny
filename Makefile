###############################################################
#
# Purpose: Makefile to build and clean all Johnny's functions
# Author.: Rafael
# Version: 0.1
# License: GPL
#
###############################################################

all:
	cd Programs/HomeCalibration && make
	cd Programs/JointDemo 		&& make
	cd Programs/TaskDemo 		&& make
	cd Programs/VisionDemo 		&& make
	cd Programs/WalkingDemo 	&& make
	cd Programs/ZMPDemo 		&& make

clean:
	cd build && make clean
	cd Programs/HomeCalibration && make clean
	cd Programs/JointDemo 		&& make clean
	cd Programs/TaskDemo 		&& make clean
	cd Programs/VisionDemo 		&& make clean
	cd Programs/WalkingDemo 	&& make clean
	cd Programs/ZMPDemo 		&& make clean
