/*
 *  WalkingPattern.h
 *
 *   Author: Rafael Nuñez
 *
 */

#ifndef _WALKING_PATTERN_H_
#define _WALKING_PATTERN_H_

	#include "stdio.h"
	#include <math.h>
	#include "JohnnyInfo.h"
	#include "RobotPose.h"

	using Pose::SIZE;

	class WalkingPattern{
		enum{ POS, VEL};
		enum{ t_i, t_ds, t_ss, t_f, KEY_POINTS_NUMBER};
		double stepDuration, doubleSupportPercentage;
		double keyTimes[KEY_POINTS_NUMBER];

		double evaluateCurrentCMLateralMovement(int stepType, double t);
		double evaluateCurrentCMFrontalMovement(int stepType, double t);
		double evaluateCurrentFootHeight(double t);
		double evaluateInterpolation(double T, double t, double x0, double x1, double xp0, double xp1);
	public:
		enum{ INITIAL_STEP, MIDDLE_STEP, FINAL_STEP, STEP_TYPES_NUMBER};
		int stepsNumber, divisionsByStep;
		double stepLenght, lateralMovement;
		double footHeight, armExtension;
		double chestFlexion;
		double walkingDirection;

		WalkingPattern();
		void calculatePose(RobotPose &robotPose, int stepType, double t);
		void setTimeParameters(double _stepDuration, double _doubleSupportPercentage);
		double getStepDuration(void);
	};

#endif
