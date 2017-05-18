#include "Johnny.h"
	
Johnny::Johnny(){
	std::cout << std::endl << "All information loaded!" << std::endl;
}

void Johnny::homePosition(double goalTime){
	robotPose.setHomePose();
	double jointsAngles[Joints::NUMBER_OF_JOINTS] = {};
	InverseKinematics::calculateCompleteInverseKinematics(jointsAngles, robotPose);
	actuators.moveAllJoints(jointsAngles, goalTime);
	wait(goalTime);
	std::cout << "Johnny is at home position!" << std::endl;
}

void Johnny::hello(bool right, double goalTime){
	if(right){
		double pose[] = {Home::rightHandHome[Pose::TX], Home::rightHandHome[Pose::TY] + 120.0, 100.0, 0.0, 0.0, 0.0};
		robotPose.setRightHandPose(pose);
	}
	else{
		double pose[] = {Home::leftHandHome[Pose::TX], Home::leftHandHome[Pose::TY] + 120.0, 100.0, 0.0, 0.0, 0.0};
		robotPose.setLeftHandPose(pose);
	}
	double jointsAngles[Joints::NUMBER_OF_JOINTS];
	InverseKinematics::calculateCompleteInverseKinematics(jointsAngles, robotPose);
	actuators.moveAllJoints(jointsAngles, goalTime);
	wait(goalTime);
	std::cout << "Johnny move its arm!" << std::endl;
}

void Johnny::walk(void){
	double sampleTime = walkingPattern.getStepDuration() / walkingPattern.divisionsByStep;
	int baseFoot = RIGHT_FOOT;
	outputFile.openFiles();

	int stepType[walkingPattern.stepsNumber + 1];
	stepType[0] = walkingPattern.INITIAL_STEP;
	for(int k = 1; k < walkingPattern.stepsNumber; k++)
		stepType[k] = walkingPattern.MIDDLE_STEP;
	stepType[walkingPattern.stepsNumber] = walkingPattern.FINAL_STEP;

	double goalAngle[Joints::NUMBER_OF_JOINTS] = {};
	walkingPattern.calculatePose(robotPose, stepType[0], 0.0);
	InverseKinematics::calculateCompleteInverseKinematics(goalAngle, robotPose);
	Stability::gravityCompensation(goalAngle, baseFoot, walkingPattern.chestFlexion);
	actuators.moveAllJoints(goalAngle, 1.0);
	wait(1.0);
	
	sensors.openOptoforcePorts();
	sensors.clearFootPorts();
	
	double t0 = getTime();
	for(int step = 0; step <= walkingPattern.stepsNumber; step++){		
		for(int division = 0; division < walkingPattern.divisionsByStep; division++){
			swipeBase(baseFoot, division, goalAngle);
			Stability::gravityCompensation(goalAngle, baseFoot, walkingPattern.chestFlexion);
			actuators.moveAllJoints(goalAngle, sampleTime);
			double goalTime = (division + 1) * sampleTime + step * walkingPattern.getStepDuration();
			sensors.getChestInformation(sampleTime);
			//sensors.getZmp(baseFoot, robotPose.singleSupport, sampleTime);
			outputFile.writeData(goalTime - sampleTime, robotPose, actuators.previousGoalAngle, sensors.sensorsData);
			walkingPattern.calculatePose(robotPose, stepType[step], (division + 1) * sampleTime);
			InverseKinematics::calculateCompleteInverseKinematics(goalAngle, robotPose);
			double delay = goalTime - (getTime() - t0);
			if(delay > 0) 
				wait(delay);
			else
				std::cout << "*";
		}
		baseFoot = -baseFoot;
		std::cout << "\tStep " << step + 1 << std::endl;
	}
	swipeBase(baseFoot, 0, goalAngle);
	Stability::gravityCompensation(goalAngle, baseFoot, walkingPattern.chestFlexion);
	actuators.moveAllJoints(goalAngle, sampleTime);
	sensors.getChestInformation(sampleTime);
	sensors.getZmp(baseFoot, robotPose.singleSupport, sampleTime);
	double currentTime = (walkingPattern.stepsNumber + 1) * walkingPattern.getStepDuration();
	outputFile.writeData(currentTime, robotPose, actuators.previousGoalAngle, sensors.sensorsData);
	
	outputFile.closeFiles();
	std::cout << "Johnny stop moving!" << std::endl << std::endl;
	sensors.closeOptoforcePorts();	
	
	homePosition(1.0);
}

void Johnny::wait(double seconds){
	double t0 = getTime(), ti = 0.0;
	do{
		usleep(100);
		ti = getTime() - t0;
	}while(ti < seconds);
}

double Johnny::getTime(void){
	struct timespec usage;
	clock_gettime(CLOCK_REALTIME, &usage);
	return(usage.tv_sec + usage.tv_nsec * RATIO_NANO2SECONDS);
}

void Johnny::swipeBase(int baseFoot, int division, double goalAngle[]){
	if((baseFoot == LEFT_FOOT && division > 0) || (baseFoot == RIGHT_FOOT && division == 0)){
		double goalAngleBackup[Joints::NUMBER_OF_JOINTS];
		std::copy(goalAngle, goalAngle + Joints::NUMBER_OF_JOINTS, goalAngleBackup);
		//                q0  q1  q2  q3  q4  q5  q6  q7  q8  q9 q10 q11 q12 q13 q14 q15 q16 q17 q18 q19 q20 q21 q22 q23 q24 q25
		double sign[] = { -1,  1,  1, -1,  1,  1,  1,  1, -1,  1,  1, -1,  1, -1,  1, -1, -1,  1,  1,  1, -1, -1,  1,  1,  1,  1 };
		for(int jointIndex = 0; jointIndex < Joints::NUMBER_OF_JOINTS; jointIndex++)
			goalAngle[jointIndex] = sign[jointIndex] * goalAngleBackup[Actuators::INDEX_LEFT[jointIndex]];
	}
}

double  Johnny::evaluateInterpolation(double T, double t, double x0, double x1, double xp0, double xp1){
	double a3 = ((xp1 - xp0) * T - 2.0 * (x1 - x0) + 2.0 * xp0 * T) / pow(T, 3.0);
	double a2 = (xp1 - xp0 - 3.0 * a3 * pow(T, 2.0)) / (2.0 * T);  	

	return(x0 + xp0 * t + a2 * pow(t, 2.0) + a3 * pow(t, 3.0));
}
