/*
 *  OutputFileManipulation.h
 *
 *   Author: Rafael Nuñez
 *
 */

#ifndef _OUTPUT_FILE_MANIPULATION_H_
#define _OUTPUT_FILE_MANIPULATION_H_

	#include <stdio.h>
	#include <iostream>
	#include <fstream>
	#include "JohnnyInfo.h"
	#include "RobotPose.h"

	class OutputFileManipulation{
	static const int NUMBER_OF_JOINTS = 26;
	std::ofstream jointsFile; 
	std::ofstream cartesiansFile;
	std::ofstream sensorsFile;

	void writeCartesiansTitle();
	void writeJointsTitle();
	void writeSensorsTitle();
	void writeCartesiansData(double ti, RobotPose robotPose);
	void writeJointsData(double ti, double jointsAngles[]);
	void writeSensorsData(double ti, SensorsData sensorsData);
 
	public:
		void openFiles(void);
		void writeData(double ti, RobotPose robotPose, double jointsAngles[], SensorsData sensorsData);
		void closeFiles(void);
	};

#endif
