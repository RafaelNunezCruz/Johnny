/*
 *  Johnny.h
 *
 *   Author: Rafael Nuñez
 *
 */

#ifndef _JOHNNY_H_
#define _JOHNNY_H_

	#include "ActuatorsManipulation.h"
	#include "SensorsManipulation.h"
	#include "OutputFileManipulation.h"
	#include "ArtificialVision.h"
	#include "WalkingPattern.h"
	#include "InverseKinematics.h"
	#include "Stability.h"
	#include "RobotPose.h"
	#include "JohnnyInfo.h"

	class Johnny{
		static const double RATIO_NANO2SECONDS = 1.0 / 1000000000.0;	
		void swipeBase(int baseFoot, int division, double goalAngle[]);
	public:
		SensorsManipulation sensors; 
		ActuatorsManipulation actuators;
		OutputFileManipulation outputFile;
		ArtificialVision artificialVision;
		WalkingPattern walkingPattern;	
		RobotPose robotPose;

		Johnny();
		void homePosition(double goalTime);
		void hello(bool right, double goalTime);
		void walk(void);
		static void wait(double seconds);
		static double getTime(void);
		static double evaluateInterpolation(double T, double t, double x0, double x1, double xp0, double xp1);
	};

#endif
