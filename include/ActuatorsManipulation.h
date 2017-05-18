/*
 *  ActuatorsManipulation.h
 *
 *   Author: Rafael Nuñez
 *
 */

#ifndef _ACTUATORS_MANIPULATION_H_
#define _ACTUATORS_MANIPULATION_H_

	#include <stdio.h>
	#include <iostream>
	#include <math.h>
	#include <unistd.h>
	#include "Robotis/LinuxCM730.h"
	#include "JohnnyInfo.h"
	using Joints::NUMBER_OF_JOINTS;

	class ActuatorsManipulation{
		static Robot::LinuxCM730 linux_cm730;
		static Robot::CM730 cm730;
 
		void writeActuatorsGains(void);
		int velocity2Word(double velocity);
		int angleWordLimit(int angleWord, int jointID);
	public:
		bool enableMovement;
		double measuredAngles[NUMBER_OF_JOINTS];
		double previousGoalAngle[NUMBER_OF_JOINTS];
		
		static int angle2Word(int jointID, double angle);
		static double word2Angle(int jointID, int word);

		ActuatorsManipulation(); 
		void moveJoint(int jointID, double goalAngle, double goalTime);
		void moveAllJoints(double goalAngle[], double goalTime);
		void readAngles(void);
		void ledState(int led);
};

#endif
