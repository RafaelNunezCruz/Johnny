/*
 *  ActuatorsManipulation.cpp
 *
 *   Author: Rafael Nuñez
 *
 */

#include "ActuatorsManipulation.h"

using namespace Actuators;
using Robot::MX28;
using Robot::CM730;
using Joints::NUMBER_OF_JOINTS;

Robot::LinuxCM730 ActuatorsManipulation::linux_cm730("/dev/ttyUSB0");
Robot::CM730 ActuatorsManipulation::cm730(&linux_cm730);

ActuatorsManipulation::ActuatorsManipulation(){
	if(cm730.Connect() == false)
		std::cout << "Fail to connect CM-730!" << std::endl;
	enableMovement = true;
	writeActuatorsGains();
	readAngles();
	std::copy(measuredAngles, measuredAngles + NUMBER_OF_JOINTS, previousGoalAngle);
}

void ActuatorsManipulation::moveJoint(int jointIndex, double goalAngle, double goalTime){
	int goalAngleWord = angleWordLimit(angle2Word(jointIndex, goalAngle), jointIndex);
	int goalVelocityWord = velocity2Word(fabs(goalAngle - previousGoalAngle[jointIndex]) / goalTime);
	if(enableMovement){
		cm730.WriteWord(jointIndex + 100, MX28::P_MOVING_SPEED_L , goalVelocityWord, 0);
		cm730.WriteWord(jointIndex + 100, MX28::P_GOAL_POSITION_L, goalAngleWord   , 0);
	}
	previousGoalAngle[jointIndex] = goalAngle;
}

void ActuatorsManipulation::moveAllJoints(double goalAngle[], double goalTime){
	int numParm = 3;
	int paramVel[Joints::NUMBER_OF_JOINTS * numParm];
	int paramPos[Joints::NUMBER_OF_JOINTS * numParm];
	for(int jointIndex = 0; jointIndex < NUMBER_OF_JOINTS; jointIndex++){
		int goalVelocityWord = velocity2Word(fabs(goalAngle[jointIndex] - previousGoalAngle[jointIndex]) / goalTime);	
		int goalAngleWord = angleWordLimit(angle2Word(jointIndex, goalAngle[jointIndex]), jointIndex);
		paramVel[jointIndex * numParm + 0] = jointIndex + 100;
		paramVel[jointIndex * numParm + 1] = CM730::GetLowByte(goalVelocityWord);
		paramVel[jointIndex * numParm + 2] = CM730::GetHighByte(goalVelocityWord);
		paramPos[jointIndex * numParm + 0] = jointIndex + 100;
		paramPos[jointIndex * numParm + 1] = CM730::GetLowByte(goalAngleWord);
		paramPos[jointIndex * numParm + 2] = CM730::GetHighByte(goalAngleWord);
		previousGoalAngle[jointIndex] = goalAngle[jointIndex];
	}
	if(enableMovement){
		cm730.SyncWrite(MX28::P_MOVING_SPEED_L , numParm, NUMBER_OF_JOINTS, paramVel);
		cm730.SyncWrite(MX28::P_GOAL_POSITION_L, numParm, NUMBER_OF_JOINTS, paramPos);
	}
}

void ActuatorsManipulation::readAngles(void){
	for(int jointIndex = 0; jointIndex < NUMBER_OF_JOINTS; jointIndex++){
		int angleWord;
		cm730.ReadWord(jointIndex + 100, MX28::P_PRESENT_POSITION_L, &angleWord, 0);
		measuredAngles[jointIndex] = word2Angle(jointIndex, angleWord); 
	}	
}

void ActuatorsManipulation::ledState(int led){
	if(led != 0)
		led = 1;
	int parametersNumber = 2;
	int param[Joints::NUMBER_OF_JOINTS * parametersNumber];
	for(int jointIndex = 0; jointIndex < NUMBER_OF_JOINTS; jointIndex++){
		param[jointIndex * parametersNumber + 0] = jointIndex + 100;
		param[jointIndex * parametersNumber + 1] = CM730::GetLowByte(led);
	}
	cm730.SyncWrite(MX28::P_LED, parametersNumber, NUMBER_OF_JOINTS, param);
}

void ActuatorsManipulation::writeActuatorsGains(void){
	for(int jointIndex = 0; jointIndex < NUMBER_OF_JOINTS; jointIndex++){
		cm730.WriteByte(jointIndex + 100, MX28::P_P_GAIN, (int)(P_GAIN[jointIndex] * RATIO_P_GAIN2WORD), 0);
		cm730.WriteByte(jointIndex + 100, MX28::P_I_GAIN, (int)(I_GAIN[jointIndex] * RATIO_I_GAIN2WORD), 0);
		cm730.WriteByte(jointIndex + 100, MX28::P_D_GAIN, (int)(D_GAIN[jointIndex] * RATIO_D_GAIN2WORD), 0);
		cm730.WriteByte(jointIndex + 100, MX28::P_TORQUE_ENABLE, 1, 0);
	}
}

int ActuatorsManipulation::angle2Word(int jointIndex, double angle){
	return((int)(HOME_ANGLE_WORD[jointIndex] + DIRECTION[jointIndex] * angle * MX28::RATIO_ANGLE2VALUE));
}

int ActuatorsManipulation::velocity2Word(double velocity){
	return (1 + (int)(velocity * RATIO_VELOCITY2WORD));
}

int ActuatorsManipulation::angleWordLimit(int angleWord, int jointIndex){
	int limitedAngleWord = angleWord;
	if(angleWord < MIN_ANGLE_WORD[jointIndex]){
		std::cout << "Goal position of actuator " << jointIndex + 100 << " is smaller than the minimum!" << std::endl;
		limitedAngleWord = MIN_ANGLE_WORD[jointIndex];
	}
	if(angleWord > MAX_ANGLE_WORD[jointIndex]){
		std::cout << "Goal position of actuator " << jointIndex + 100 << " is bigger than the maximum!" << std::endl;
		limitedAngleWord = MAX_ANGLE_WORD[jointIndex];
	}
	return limitedAngleWord;
}

double ActuatorsManipulation::word2Angle(int jointIndex, int word){
	return(MX28::RATIO_VALUE2ANGLE * DIRECTION[jointIndex] * (word - HOME_ANGLE_WORD[jointIndex]));
}
