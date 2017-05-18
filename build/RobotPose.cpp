/*
 *  RobotPose.cpp
 *
 *   Author: Rafael Nuñez
 *
 */

#include "RobotPose.h"

using namespace Dimensions;
using namespace Pose;
using namespace Home;
using std::copy;

RobotPose::RobotPose(){
	setHomePose();
}

void RobotPose::setHomePose(void){
	copy(centerMassHome, centerMassHome + SIZE, centerMass);
	copy(leftFootHome, leftFootHome + SIZE, leftFoot);
	copy(chestHome, chestHome + SIZE, chest);
	copy(headHome, headHome + SIZE, head);
	copy(rightHandHome, rightHandHome + SIZE,rightHand);
	copy(leftHandHome, leftHandHome + SIZE, leftHand);	
	singleSupport = false;
}

void RobotPose::setCenterMassPosition(double centerMassPose[]){
	copy(centerMassPose, centerMassPose + SIZE, centerMass);
	centerMass[RX] = 0.0;
	centerMass[RY] = 0.0;
	centerMass[RZ] = 0.0;
	setChestOrientation(chest);
}

void RobotPose::setChestOrientation(double chestPose[]){
	chest[RX] = chestPose[RX];
	chest[RY] = chestPose[RY];
	chest[RZ] = chestPose[RZ];
	chest[TX] = centerMass[TX] - COLUMN_SUP * sin(chest[RZ] * RATIO_DEGREES2RADIANS);
	chest[TY] = centerMass[TY] - R_HIP_CM_DY + COLUMN_INF + (COLUMN_MED + COLUMN_SUP * cos(chest[RZ] * RATIO_DEGREES2RADIANS)) * cos(chest[RX] * RATIO_DEGREES2RADIANS);
	chest[TZ] = centerMass[TZ] - R_HIP_CM_DZ  + (COLUMN_MED + COLUMN_SUP * cos(chest[RZ] * RATIO_DEGREES2RADIANS)) * sin(chest[RX] * RATIO_DEGREES2RADIANS);
	setHeadOrientation(head);
}

void RobotPose::setHeadOrientation(double headPose[]){
	head[RX] = headPose[RX];
	head[RY] = headPose[RY];
	head[RZ] = chest[RZ];
	head[TX] = 0.0; // TODO: calculate absolute x position of the head (to be used for grasp task)
	head[TY] = 0.0; // TODO: calculate absolute y position of the head (to be used for grasp task)
	head[TZ] = 0.0; // TODO: calculate absolute z position of the head (to be used for grasp task)
}

void RobotPose::setLeftFootPose(double leftFootPose[]){
	copy(leftFootPose, leftFootPose + SIZE, leftFoot);
	singleSupport = leftFootPose[TY] > 0.0;
}

void RobotPose::setRightHandPose(double rightHandPose[]){
	copy(rightHandPose, rightHandPose + SIZE, rightHand);
	rightHand[RX] = 0.0; // TODO: calculate absolute x orientation of the right hand (for completeness)
	rightHand[RZ] = 0.0; // TODO: calculate absolute z orientation of the right hand (for completeness)
}

void RobotPose::setLeftHandPose(double leftHandPose[]){
	copy(leftHandPose, leftHandPose + SIZE, leftHand);
	leftHand[RX] = 0.0; // TODO: calculate absolute x orientation of the left hand (for completeness)
	leftHand[RZ] = 0.0; // TODO: calculate absolute z orientation of the left hand (for completeness)
}
