/*
 *  WalkingPattern.cpp
 *
 *   Author: Rafael Nuñez
 *
 */

#include "WalkingPattern.h"

WalkingPattern::WalkingPattern(){
	setTimeParameters(0.3, 0.20);
	stepsNumber = 20;
	divisionsByStep = (int)(stepDuration / 0.02);
	stepLenght = 20.0;
	lateralMovement = 15.0; 
	footHeight = 8.0;
	chestFlexion = 0.0;
	armExtension = Dimensions::UPPER_ARM_LENGHT + Dimensions::FOREARM_LENGHT - 5.0;
}

void WalkingPattern::calculatePose(RobotPose &robotPose, int stepType, double t){
	using namespace Home;
	using namespace Pose;
	using namespace Dimensions;

	double centerMassPose[SIZE] = {	evaluateCurrentCMLateralMovement(stepType, t),
									0.0,
									evaluateCurrentCMFrontalMovement(stepType, t)	};
	centerMassPose[TY] = R_HIP_CM_DY + sqrt(pow(THIGH_LENGHT + SHANK_LENGHT, 2.0) - pow(centerMassPose[TX] - R_HIP_CM_DX, 2.0) - pow(centerMassPose[TZ] - R_HIP_CM_DZ, 2.0));
	robotPose.setCenterMassPosition(centerMassPose);

	double lFootPose[SIZE] = {	leftFootHome[TX], 
								evaluateCurrentFootHeight(t), 
								2.0 * (robotPose.centerMass[TZ] - centerMassHome[TZ])	};
	robotPose.setLeftFootPose(lFootPose);

	double rHandPose[SIZE] = {	robotPose.chest[TX] - R_SHOULDER_CHEST_DX, 
								robotPose.chest[TY] - sqrt(pow(armExtension, 2.0) - pow(robotPose.leftFoot[TZ] - robotPose.chest[TZ], 2.0)),
								robotPose.leftFoot[TZ]	};
	robotPose.setRightHandPose(rHandPose);

	double lHandPose[SIZE] = {	robotPose.chest[TX] + CHEST_L_SHOULDER_DX, 
								robotPose.chest[TY] - sqrt(pow(armExtension, 2.0) - pow(robotPose.chest[TZ], 2.0)),
								0.0	};
	robotPose.setLeftHandPose(lHandPose);
}

void WalkingPattern::setTimeParameters(double singleSupportTime, double doubleSupportTime){
	stepDuration = singleSupportTime + doubleSupportTime;
	keyTimes[t_i]  = 0.0;
	keyTimes[t_ds] = doubleSupportTime / 2.0;
	keyTimes[t_ss] = singleSupportTime + keyTimes[t_ds];
	keyTimes[t_f]  = stepDuration;
}

double WalkingPattern::getStepDuration(void){
	return stepDuration;
}

double WalkingPattern::evaluateCurrentCMLateralMovement(int stepType, double t){
	using namespace Home;
	using namespace Pose;

	double maxLateralMovement = centerMassHome[TX] - lateralMovement;
	double currentLateralMovement = maxLateralMovement;
	double initialLateralVelocity = -1.5 * lateralMovement / keyTimes[t_ds];
	double finalLateralVelocity = -initialLateralVelocity;

	//if(stepType == INITIAL_STEP)
	//	initialLateralVelocity = 0.0;

	if(stepType == FINAL_STEP)
		finalLateralVelocity = 0.0;

	if(keyTimes[t_i] <= t && t <= stepDuration / 2.0)
		currentLateralMovement = evaluateInterpolation(stepDuration / 2.0, t, centerMassHome[TX], maxLateralMovement, initialLateralVelocity, 0.0);

	if(stepDuration / 2.0 < t && t <= keyTimes[t_f])
		currentLateralMovement = evaluateInterpolation(stepDuration / 2.0, t - stepDuration / 2.0, maxLateralMovement, centerMassHome[TX], 0.0, finalLateralVelocity);

	return currentLateralMovement;
}

double WalkingPattern::evaluateCurrentCMFrontalMovement(int stepType, double t){
	using namespace Home;
	using namespace Pose;

	double minFrontalMovement = centerMassHome[TZ] - stepLenght / 2.0;
	double maxFrontalMovement = centerMassHome[TZ] + stepLenght / 2.0;
	double currentFrontalMovement;

	if(stepType == INITIAL_STEP)
		minFrontalMovement = centerMassHome[TZ];

	if(stepType == FINAL_STEP)
		maxFrontalMovement = centerMassHome[TZ];

	if(t <= keyTimes[t_ds])
		currentFrontalMovement = minFrontalMovement;
	else
		if(keyTimes[t_ds] < t && t <= keyTimes[t_ss])
			currentFrontalMovement = evaluateInterpolation(keyTimes[t_ss] - keyTimes[t_ds], t - keyTimes[t_ds], minFrontalMovement, maxFrontalMovement, 0.0, 0.0);
		else
			currentFrontalMovement = maxFrontalMovement;

	return currentFrontalMovement;
}

double WalkingPattern::evaluateCurrentFootHeight(double t){
	double height = 0.0;

	if(keyTimes[t_ds] < t && t <= (keyTimes[t_ds] + keyTimes[t_ss]) / 2)
		height = evaluateInterpolation((keyTimes[t_ss] - keyTimes[t_ds]) / 2, t - keyTimes[t_ds], 0.0, footHeight, 0.0, 0.0);
	else
		if((keyTimes[t_ds] + keyTimes[t_ss]) / 2 < t && t <= keyTimes[t_ss])
			height = evaluateInterpolation((keyTimes[t_ss] - keyTimes[t_ds]) / 2, t - (keyTimes[t_ds] + keyTimes[t_ss]) / 2 , footHeight, 0.0, 0.0, 0.0);

	return height;
}

double WalkingPattern::evaluateInterpolation(double T, double t, double x0, double x1, double xp0, double xp1){
	double a3 = ((xp1 - xp0) * T - 2.0 * (x1 - x0) + 2.0 * xp0 * T) / pow(T, 3.0);
	double a2 = (xp1 - xp0 - 3.0 * a3 * pow(T, 2.0)) / (2.0 * T);  	

	return(x0 + xp0 * t + a2 * pow(t, 2.0) + a3 * pow(t, 3.0));
}
