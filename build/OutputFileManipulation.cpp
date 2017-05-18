#include "OutputFileManipulation.h"

void OutputFileManipulation::openFiles(void){
	jointsFile.open("joints.txt");
	cartesiansFile.open("cartesians.txt");
	sensorsFile.open("sensors.txt");
	writeCartesiansTitle();
	writeJointsTitle();
	writeSensorsTitle();
}

void OutputFileManipulation::writeData(double ti, RobotPose robotPose, double jointsAngles[], SensorsData sensorsData){
	writeCartesiansData(ti, robotPose);
	writeJointsData(ti, jointsAngles);
	writeSensorsData(ti, sensorsData);
}

void OutputFileManipulation::closeFiles(void){
	if(jointsFile.is_open())
    	jointsFile.close();
	if(cartesiansFile.is_open())
    	cartesiansFile.close();
	if(sensorsFile.is_open())
    	sensorsFile.close();
}

void OutputFileManipulation::writeCartesiansTitle(){
	if(cartesiansFile.is_open()){
		cartesiansFile << "t \t";
		for(int k = 0; k < 3; k++)
			cartesiansFile << "CM[" << k+1 << "] \t";
		for(int k = 0; k < 3; k++)
			cartesiansFile << "CH[" << k+1 << "] \t";
		for(int k = 0; k < 3; k++)
			cartesiansFile << "LF[" << k+1 << "] \t";
		for(int k = 0; k < 3; k++)
			cartesiansFile << "RH[" << k+1 << "] \t";
		for(int k = 0; k < 3; k++)
			cartesiansFile << "LH[" << k+1 << "] \t";
		cartesiansFile << std::endl;
	}
	else
		std::cout << "Unable to open cartesians.txt" << std::endl;
}

void OutputFileManipulation::writeJointsTitle(){
	if(jointsFile.is_open()){
		jointsFile << "t \t";
		for(int k = 0; k < NUMBER_OF_JOINTS; k++)
			jointsFile << "Q[" << k+100 << "] \t";
		jointsFile << std::endl;
	}
	else
		std::cout << "Unable to open joints.txt" << std::endl;
}

void OutputFileManipulation::writeSensorsTitle(){
	if(sensorsFile.is_open()){
		sensorsFile << "t \t";
		for(int k = 0; k < 3; k++)
			sensorsFile << "CI[" << k+1 << "] \t";
		for(int k = 0; k < 3; k++)
			sensorsFile << "CV[" << k+1 << "] \t";
		for(int k = 0; k < 3; k++)
			sensorsFile << "ZMP[" << k+1 << "] \t";
		sensorsFile << std::endl;
	}
	else
		std::cout << "Unable to open sensors.txt" << std::endl;
}

void OutputFileManipulation::writeCartesiansData(double ti, RobotPose robotPose){
	if(cartesiansFile.is_open()){
		cartesiansFile << ti << " \t";
		for(int k = 0; k < 3; k++)
			cartesiansFile << robotPose.centerMass[k] << " \t";
		for(int k = 0; k < 3; k++)
			cartesiansFile << robotPose.chest[k] << " \t";
		for(int k = 0; k < 3; k++)
			cartesiansFile << robotPose.leftFoot[k] << " \t";
		for(int k = 0; k < 3; k++)
			cartesiansFile << robotPose.rightHand[k] << " \t";
		for(int k = 0; k < 3; k++)
			cartesiansFile << robotPose.leftHand[k] << " \t";
		cartesiansFile << std::endl;
	}
	else
		std::cout << "Unable to open cartesian.txt" << std::endl;
}

void OutputFileManipulation::writeJointsData(double ti, double jointsAngles[]){
	if(jointsFile.is_open()){
		jointsFile << ti << " \t";
		for(int k = 0; k < NUMBER_OF_JOINTS; k++)
			jointsFile << jointsAngles[k] << " \t";
		jointsFile << std::endl;
	}
	else
		std::cout << "Unable to open joints.txt" << std::endl;
}

void OutputFileManipulation::writeSensorsData(double ti, SensorsData sensorsData){
	if(sensorsFile.is_open()){
		sensorsFile << ti << " \t";
		for(int k = 0; k < 3; k++)
			sensorsFile << sensorsData.chestInclination.position[k] << " \t";
		for(int k = 0; k < 3; k++)
			sensorsFile << sensorsData.chestAngularVelocity.position[k] << " \t";
		for(int k = 0; k < 3; k++)
			sensorsFile << sensorsData.zmp.position[k] << " \t";
		sensorsFile << std::endl;
	}
	else
		std::cout << "Unable to open sensors.txt" << std::endl;
}
