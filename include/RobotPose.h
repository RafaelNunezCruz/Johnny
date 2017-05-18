/*
 *  RobotPose.h
 *
 *   Author: Rafael Nuñez
 *
 */

#ifndef _ROBOT_POSE_H_
#define _ROBOT_POSE_H_

	#include <iostream>
	#include <math.h>
	#include "JohnnyInfo.h"
	using Pose::SIZE;

	class RobotPose{	
		static const double RATIO_DEGREES2RADIANS = 3.1416 / 180.0;
	public:		
		double centerMass[SIZE], chest[SIZE], head[SIZE];
		double leftFoot[SIZE], rightHand[SIZE], leftHand[SIZE];	
		bool singleSupport;

		RobotPose(void);
		void setHomePose(void);
		void setCenterMassPosition(double centerMassPose[]);
		void setChestOrientation(double chestPose[]);
		void setHeadOrientation(double headPose[]);
		void setLeftFootPose(double leftFootPose[]);
		void setRightHandPose(double rightHandPose[]);
		void setLeftHandPose(double leftHandPose[]);
	};

#endif
