/*
 *  SensorsManipulation.h
 *
 *   Author: Rafael Nuñez
 *
 */

#ifndef _SENSORS_MANIPULATION_H_
#define _SENSORS_MANIPULATION_H_

	#include <stdio.h>
	#include <iostream>
	#include <math.h>
	#include "Robotis/LinuxCM730.h"
	#include <SerialPort.h> 
	#include "JohnnyInfo.h"
#include <iostream>
#include <string>
	using namespace Sensors;

	class SensorsManipulation{ 
		static const double RATIO_RADIANS2DEGREES = 180.0 / 3.1416;
		static const double RATIO_WORD2ACCELERATION = 4.0 * 9.81 / (1023.0 - 512.0);
		static const double RATIO_WORD2VELOCITY = -1600.0 / (1023.0 - 512.0);
		static const int CENTER_VALUE = 512;
		static Robot::LinuxCM730 linux_cm730;
		static Robot::CM730 cm730;	
		static SerialPort serialPortRight;
		static SerialPort serialPortLeft;
		
		static void initializeOptoforcePort(SerialPort &serialPort);
		static void clearOptoforcePort(SerialPort &serialPort);
		static bool readOptoforceSensor(SerialPort &serialPort, int data[]);
		static void optoforceDataConversion(double value[], int data[], int side);
		static void optoforceData2zmp(double zmp[], double sensor[], double normalVector[]);
		bool getParcialZmp(int side, double parcialZmp[]);
	public:			
		SensorsData sensorsData;
		bool isOptoforcePortsOpened;

		SensorsManipulation();
		~SensorsManipulation();
		void openOptoforcePorts();
		void closeOptoforcePorts();
		void getChestInformation(double sampleTime);
		void getZmp(int side, bool singleSupport, double sampleTime);
		void clearFootPorts();
	};

#endif
