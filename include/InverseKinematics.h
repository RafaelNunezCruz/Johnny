/*
 *  InverseKinematics.h
 *
 *   Author: Rafael Nuñez
 *
 */

#ifndef _INVERSE_KINEMATICS_H_
#define _INVERSE_KINEMATICS_H_

	#include "stdio.h"
	#include <iostream>	
	#include <math.h>
	#include "JohnnyInfo.h"
	#include "RobotPose.h"

	class InverseKinematics{
		static const double RATIO_DEGREES2RADIANS = 3.1416 / 180.0;
		static const double RATIO_RADIANS2DEGREES = 180.0 / 3.1416;

		static void lawOfCosines(double &alpha, double &beta, double a, double b, double c);
		static double getPositionMagnitude(double pose[]);
		InverseKinematics(){}
	public:
		static void calculateCompleteInverseKinematics(double jointsAngles[], RobotPose robotPose);
		static void calculateHipInverseKinematics(double jointsAngles[], RobotPose robotPose);
		static void calculateLeftFootInverseKinematics(double jointsAngles[], RobotPose robotPose);
		static void calculateChestInverseKinematics(double jointsAngles[], RobotPose robotPose);
		static void calculateHeadInverseKinematics(double jointsAngles[], RobotPose robotPose);
		static void calculateRightHandInverseKinematics(double jointsAngles[], RobotPose robotPose);
		static void calculateLeftHandInverseKinematics(double jointsAngles[], RobotPose robotPose);
	};

#endif
