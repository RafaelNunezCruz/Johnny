#include "Johnny.h" 

double evaluateInterpolation(double T, double t, double x0, double x1, double xp0, double xp1){
	double a3 = ((xp1 - xp0) * T - 2.0 * (x1 - x0) + 2.0 * xp0 * T) / pow(T, 3.0);
	double a2 = (xp1 - xp0 - 3.0 * a3 * pow(T, 2.0)) / (2.0 * T);  	

	return(x0 + xp0 * t + a2 * pow(t, 2.0) + a3 * pow(t, 3.0));
}

int main(int argc,char *argv[]) {
	using namespace Dimensions;
	using namespace Pose;

	Johnny johnny;
	
	std::cout << "\n=== Johnny Task Demo ===\n\n";
	
	johnny.homePosition(1.0);	
	
	if (argc != 4) {
		std::cout 
		<< "\nTo run this program you should define:\n"
		<< "kp (10^3)\n"
		<< "kd (10^3)\n"
		<< "Amplitude (int degrees)\n"
		<< "Hint: ./TaskDemo 375 30 15\n\n";
		return 0;		
	}
	
	johnny.outputFile.openFiles();

	double cmPose[SIZE], chestPose[SIZE], rHandPose[SIZE], lHandPose[SIZE];
	std::copy(Home::centerMassHome, Home::centerMassHome + SIZE, cmPose);
	std::copy(Home::chestHome, Home::chestHome + SIZE, chestPose);
	std::copy(Home::rightHandHome , Home::rightHandHome  + SIZE, rHandPose);
	std::copy(Home::leftHandHome  , Home::leftHandHome   + SIZE, lHandPose);
	
	double jointsAngles[Joints::NUMBER_OF_JOINTS] = {};
	
	double cycleTime = 3.0;
	double totalTime = 3.0 * cycleTime;
	double incTime = 0.02;
	double refTime = 1.0;
	
	double kp = atoi(argv[1]) / 1000.0;
	double kd = atoi(argv[2]) / 1000.0;
	double amplitude = atoi(argv[3]);
	
	double zmpRef[3] = {}, _zmpRef[3] = {};

	for(double t = 0.0 ; t <= totalTime; t = t + incTime){
		if(t > 0.0)
			std::copy(zmpRef, zmpRef + 3, _zmpRef);
		
		johnny.sensors.getZmp(1, johnny.robotPose.singleSupport, incTime);
		
//		zmpRef[TX] = - amplitude * sin(2.0 * 3.1416 * t / cycleTime);
//		if(t < refTime)
//			zmpRef[TZ] = evaluateInterpolation(refTime, t, -18.0, 0.0, 0.0, 0.0);
//		else
//			zmpRef[TZ] = 0.0;
			
//		cmPose[TX] = johnny.robotPose.centerMass[TX] 
//					- incTime * kp * (	zmpRef[TX] - johnny.sensors.sensorsData.zmp[TX]) 
//					- 			kd * (	zmpRef[TX] - _zmpRef[TX] - (johnny.sensors.sensorsData.zmp[TX] - johnny.sensors.sensorsData._zmp[TX]));
					
//		cmPose[TZ] = johnny.robotPose.centerMass[TZ] 
//					- incTime * kp * ( johnny.sensors.sensorsData.zmp[TZ] - zmpRef[TZ] ) 
//					- 			kd * ( johnny.sensors.sensorsData.zmp[TZ] - johnny.sensors.sensorsData._zmp[TZ] - (zmpRef[TZ] - _zmpRef[TZ]));

			
//		cmPose[TY] = R_HIP_CM_DY + sqrt(pow(THIGH_LENGHT + SHANK_LENGHT, 2.0) - pow(cmPose[TX] - R_HIP_CM_DX, 2.0) - pow(cmPose[TZ] - R_HIP_CM_DZ, 2.0));
//		johnny.robotPose.setCenterMassPosition(cmPose);
		
//		rHandPose[TX] = johnny.robotPose.chest[TX] - R_SHOULDER_CHEST_DX;
//		rHandPose[TY] = johnny.robotPose.chest[TY] - UPPER_ARM_LENGHT - FOREARM_LENGHT;
//		rHandPose[TZ] = johnny.robotPose.chest[TZ];
//		johnny.robotPose.setRightHandPose(rHandPose);

//		lHandPose[TX] = johnny.robotPose.chest[TX] + R_SHOULDER_CHEST_DX;
//		lHandPose[TY] = johnny.robotPose.chest[TY] - UPPER_ARM_LENGHT - FOREARM_LENGHT;
//		lHandPose[TZ] = johnny.robotPose.chest[TZ];
//		johnny.robotPose.setLeftHandPose(lHandPose);
		

		double dtz = 5.0;
		if(t < refTime)
			chestPose[RX] = evaluateInterpolation(refTime, t, Home::chestHome[RX], Home::chestHome[RX] + dtz, 0.0, 0.0);
		else
			chestPose[RX] = Home::chestHome[RX] + dtz;
			
		johnny.robotPose.setChestOrientation(chestPose);

		InverseKinematics::calculateCompleteInverseKinematics(jointsAngles, johnny.robotPose);
		johnny.actuators.moveAllJoints(jointsAngles, incTime);
		johnny.outputFile.writeData(t, johnny.robotPose, johnny.actuators.previousGoalAngle, johnny.sensors.sensorsData);
		johnny.wait(incTime);
	}
	
	johnny.outputFile.closeFiles();		
	johnny.homePosition(1.0);
    return 0;
}
