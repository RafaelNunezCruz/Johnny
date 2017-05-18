/*
 *  InverseKinematics.cpp
 *
 *   Author: Rafael Nuñez
 *
 */

#include "InverseKinematics.h"

using namespace Joints;
using namespace Dimensions;
using namespace Pose;
using std::copy;

void InverseKinematics::calculateCompleteInverseKinematics(double jointsAngles[], RobotPose robotPose){
	calculateHipInverseKinematics(jointsAngles, robotPose);
	calculateLeftFootInverseKinematics(jointsAngles, robotPose);
	calculateChestInverseKinematics(jointsAngles, robotPose);
	calculateHeadInverseKinematics(jointsAngles, robotPose);
	calculateRightHandInverseKinematics(jointsAngles, robotPose);
	calculateLeftHandInverseKinematics(jointsAngles, robotPose);
}

void InverseKinematics::calculateHipInverseKinematics(double jointsAngles[], RobotPose robotPose){
	// TODO: add the effect of changing walking direction
	double rightHip2Ankle[SIZE] = {};
	rightHip2Ankle[TX] = robotPose.centerMass[TX] - R_HIP_CM_DX;
	rightHip2Ankle[TY] = robotPose.centerMass[TY] - R_HIP_CM_DY;
	rightHip2Ankle[TZ] = robotPose.centerMass[TZ] - R_HIP_CM_DZ;
	double positionMagnitude = getPositionMagnitude(rightHip2Ankle);
	if(positionMagnitude > SHANK_LENGHT + THIGH_LENGHT + 0.001)
		std::cout << "Can not achieve right leg extension, joint values will be NaN! " << positionMagnitude << std::endl;
	double alpha, beta;
	lawOfCosines(alpha, beta, SHANK_LENGHT, THIGH_LENGHT, positionMagnitude);
	double gama = atan2(rightHip2Ankle[TZ], rightHip2Ankle[TY]) * RATIO_RADIANS2DEGREES;	
	jointsAngles[R_ANKLE_EVERSION] = atan2(-rightHip2Ankle[TX], rightHip2Ankle[TY]) * RATIO_RADIANS2DEGREES;
	jointsAngles[R_ANKLE_FLEXION] = gama + beta;
	jointsAngles[R_KNEE_FLEXION] = alpha + beta;
	jointsAngles[R_HIP_ABDUCTION] = -jointsAngles[R_ANKLE_EVERSION];
	jointsAngles[R_HIP_FLEXION] = alpha - gama;
	jointsAngles[R_HIP_ROTATION] = 0.0;
}

void InverseKinematics::calculateLeftFootInverseKinematics(double jointsAngles[], RobotPose robotPose){
	// TODO: add the effect of changing walking direction
	double leftFoot2Hip[SIZE] = {};
	leftFoot2Hip[TX] = robotPose.centerMass[TX] + CM_L_HIP_DX - robotPose.leftFoot[TX];
	leftFoot2Hip[TY] = robotPose.centerMass[TY] + CM_L_HIP_DY - robotPose.leftFoot[TY];
	leftFoot2Hip[TZ] = robotPose.centerMass[TZ] + CM_L_HIP_DZ - robotPose.leftFoot[TZ];
	leftFoot2Hip[RX] = robotPose.leftFoot[RX];
	double positionMagnitude = getPositionMagnitude(leftFoot2Hip);
	if(positionMagnitude > SHANK_LENGHT + THIGH_LENGHT + 0.001)
		std::cout << "Can not achieve left leg extension, joint values will be NaN! " << positionMagnitude << std::endl;
	double alpha, beta;
	lawOfCosines(alpha, beta, SHANK_LENGHT, THIGH_LENGHT, positionMagnitude);
	double gama = atan2(leftFoot2Hip[TZ], leftFoot2Hip[TY]) * RATIO_RADIANS2DEGREES;	
	jointsAngles[L_HIP_ROTATION] = 0.0;
	jointsAngles[L_HIP_FLEXION] = alpha - gama;
	jointsAngles[L_HIP_ABDUCTION] = atan2(leftFoot2Hip[TX], leftFoot2Hip[TY]) * RATIO_RADIANS2DEGREES;
	jointsAngles[L_KNEE_FLEXION] = alpha + beta;
	jointsAngles[L_ANKLE_FLEXION] = gama + beta - leftFoot2Hip[RX];
	jointsAngles[L_ANKLE_EVERSION] = -jointsAngles[L_HIP_ABDUCTION];
}

void InverseKinematics::calculateChestInverseKinematics(double jointsAngles[], RobotPose robotPose){
	jointsAngles[COLUMN_FLEXION] = robotPose.chest[RX];
	jointsAngles[COLUMN_ABDUCTION] = robotPose.chest[RZ];
}

void InverseKinematics::calculateHeadInverseKinematics(double jointsAngles[], RobotPose robotPose){
	jointsAngles[HEAD_ROTATION] = robotPose.head[RY];
	jointsAngles[HEAD_FLEXION] = robotPose.head[RX] - robotPose.chest[RX];
}

void InverseKinematics::calculateRightHandInverseKinematics(double jointsAngles[], RobotPose robotPose){
	double rightHand2Shoulder[SIZE] = {};
	rightHand2Shoulder[TX] = robotPose.chest[TX] - R_SHOULDER_CHEST_DX * cos(robotPose.chest[RZ] * RATIO_DEGREES2RADIANS) - robotPose.rightHand[TX];
	rightHand2Shoulder[TY] = robotPose.chest[TY] + R_SHOULDER_CHEST_DX * sin(robotPose.chest[RZ] * RATIO_DEGREES2RADIANS) - robotPose.rightHand[TY];
	rightHand2Shoulder[TZ] = robotPose.chest[TZ] - robotPose.rightHand[TZ];
	double alpha, beta;
	double positionMagnitude = getPositionMagnitude(rightHand2Shoulder);
	if(positionMagnitude > FOREARM_LENGHT + UPPER_ARM_LENGHT + 0.001)
		std::cout << "Can not achieve right arm extension, joint values will be NaN! " << positionMagnitude << std::endl;
    lawOfCosines(alpha, beta, FOREARM_LENGHT, UPPER_ARM_LENGHT, positionMagnitude);
	double gama = atan2(rightHand2Shoulder[TZ], rightHand2Shoulder[TY]) * RATIO_RADIANS2DEGREES;
    jointsAngles[R_SHOULDER_FLEXION] = gama + beta;
    jointsAngles[R_SHOULDER_ABDUCTION] = atan2(rightHand2Shoulder[TX], rightHand2Shoulder[TY]) * RATIO_RADIANS2DEGREES;
	jointsAngles[R_SHOULDER_ROTATION] = 0;
    jointsAngles[R_ELBOW] = - alpha - beta;
    jointsAngles[R_HAND] = 0.0;
}

void InverseKinematics::calculateLeftHandInverseKinematics(double jointsAngles[], RobotPose robotPose){
	double leftHand2Shoulder[SIZE] = {};
	leftHand2Shoulder[TX] = robotPose.chest[TX] + CHEST_L_SHOULDER_DX * cos(robotPose.chest[RZ] * RATIO_DEGREES2RADIANS) - robotPose.leftHand[TX];
	leftHand2Shoulder[TY] = robotPose.chest[TY] - CHEST_L_SHOULDER_DX * sin(robotPose.chest[RZ] * RATIO_DEGREES2RADIANS) - robotPose.leftHand[TY];
	leftHand2Shoulder[TZ] = robotPose.chest[TZ] - robotPose.leftHand[TZ];
	double alpha, beta;
	double positionMagnitude = getPositionMagnitude(leftHand2Shoulder);
	if(positionMagnitude > FOREARM_LENGHT + UPPER_ARM_LENGHT + 0.001)
		std::cout << "Can not achieve left arm extension, joint values will be NaN! " << positionMagnitude << std::endl;
    lawOfCosines(alpha, beta, FOREARM_LENGHT, UPPER_ARM_LENGHT, positionMagnitude);
	double gama = atan2(leftHand2Shoulder[TZ], leftHand2Shoulder[TY]) * RATIO_RADIANS2DEGREES;
    jointsAngles[L_SHOULDER_FLEXION] = gama + beta;
    jointsAngles[L_SHOULDER_ABDUCTION] = atan2(leftHand2Shoulder[TX], leftHand2Shoulder[TY]) * RATIO_RADIANS2DEGREES;
	jointsAngles[L_SHOULDER_ROTATION] = 0;
    jointsAngles[L_ELBOW] = - alpha - beta;
    jointsAngles[L_HAND] = 0.0;
}

void InverseKinematics::lawOfCosines(double &alpha, double &beta, double a, double b, double c){
    alpha = acos((pow(c, 2) + pow(b, 2) - pow(a, 2)) / (2 * c * b)) * RATIO_RADIANS2DEGREES;
	beta  = acos((pow(a, 2) + pow(c, 2) - pow(b, 2)) / (2 * a * c)) * RATIO_RADIANS2DEGREES;
}
		
double InverseKinematics::getPositionMagnitude(double pose[]){
	return sqrt(pow(pose[TX], 2) + pow(pose[TY], 2) + pow(pose[TZ], 2));
}
