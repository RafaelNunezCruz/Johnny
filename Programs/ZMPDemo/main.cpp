#include "Johnny.h" 

int main(int argc,char *argv[]) {
	Johnny johnny;
	
	std::cout << "\n=== Johnny ZMP Demo ===\n\n";

	johnny.homePosition(1.0);
	
	if (argc != 7) {
		std::cout 
		<< "\nTo run this program you should define:\n"
		<< "X proporcional gain (10^3)\n"
		<< "X integral gain (10^3)\n"
		<< "X derivative gain (10^3)\n"
		<< "Z proporcional gain (10^3)\n"
		<< "Z integral gain (10^3)\n"
		<< "Z derivative gain (10^3)\n"
		<< "Hint: ./ZmpDemo 800 0 700 200 0 190\n\n";
		return 0;		
	}
	
	johnny.outputFile.openFiles();
	
	int cyclesNumber = 3;
	double cycleTime = 4.0;
	double sampleTime = 0.02;
	
	double jointsAngles[Joints::NUMBER_OF_JOINTS] = {};
	
	Control ctrlX, ctrlZ;
	
	ctrlX.pGain = sampleTime * (double)atoi(argv[1]) / 1000.0;
	ctrlX.iGain = sampleTime * (double)atoi(argv[2]) / 1000.0;
	ctrlX.dGain = sampleTime * (double)atoi(argv[3]) / 1000.0;
	ctrlZ.pGain = sampleTime * (double)atoi(argv[4]) / 1000.0;
	ctrlZ.iGain = sampleTime * (double)atoi(argv[5]) / 1000.0;
	ctrlZ.dGain = sampleTime * (double)atoi(argv[6]) / 1000.0;
	
	FilteredSignal zmpReference;
	
	johnny.sensors.clearFootPorts();
	
	double t0 = johnny.getTime();
	
	for(int k = 0; k < cyclesNumber; k++){
		for(double t = 0.0; t < cycleTime; t = t + sampleTime){
			johnny.sensors.getChestInformation(sampleTime);
			johnny.sensors.getZmp(1, false, sampleTime);
			
			double amplitude = 8.0;
			double finalOffset = 3.0;
			double initialOffset = finalOffset - amplitude;
			double offset = finalOffset;
			if(k == 0 && t < cycleTime / 2.0) 
				offset = johnny.evaluateInterpolation(cycleTime / 2.0, t, initialOffset, finalOffset, 0.0, 0.0);
			
			double reference[] = {	amplitude * cos(2.0 * 3.1416 * t / cycleTime) + offset,
									0.0,
									0.0};
			if(k == 0)
				reference[Pose::TZ] = johnny.evaluateInterpolation(cycleTime, t, -36.0, 0.0, 0.0, 0.0); 
				
			zmpReference.addSample(reference, sampleTime);
			
			Stability::zmpControl(	jointsAngles, 
									johnny.sensors.sensorsData.zmp, 
									zmpReference, 
									ctrlX, ctrlZ);
												
			johnny.actuators.moveAllJoints(jointsAngles, sampleTime);
			
			std::copy(reference, reference + 3, johnny.sensors.sensorsData.chestAngularVelocity.position);
			
			double currentTime = johnny.getTime() - t0;
			
			johnny.outputFile.writeData(currentTime, 
										johnny.robotPose, 
										jointsAngles, 
										johnny.sensors.sensorsData);
											
			double goalTime = k * cycleTime + t;
			double delay = goalTime - currentTime + sampleTime;
			
			if(delay > 0.0) 
				johnny.wait(delay);
			else
				std::cout << "*";
			
		}		
	}
	
	johnny.outputFile.closeFiles();		
	johnny.homePosition(1.0);
    return 1;
}
