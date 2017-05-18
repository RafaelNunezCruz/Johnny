#include "Johnny.h" 

int main(int argc,char *argv[]) {
	std::cout << "\n=== Johnny Walking Demo ===\n\n";
	Johnny johnny;

	if (argc != 8) {
		std::cout 
		<< "\nTo run this program you should define:\n"
		<< "Step lenght (mm)\n"
		<< "Single support duration (ms)\n"
		<< "Double support duration (ms)\n"
		<< "Lateral movement (mm)\n"
		<< "Foot Height (mm)\n"
		<< "Chest Flexion (deg * 10)\n"
		<< "Number of steps\n\n"
		<< "Hint: ./WalkingDemo 25 270 470 30 6 0 10\n\n";
		johnny.homePosition(1.0);
		return 0;		
	}
	
	johnny.walkingPattern.stepLenght = atoi(argv[1]);
	double ss = (double)atoi(argv[2]) / 1000.0;
	double ds = (double)atoi(argv[3]) / 1000.0;
	johnny.walkingPattern.setTimeParameters(ss, ds);	
	johnny.walkingPattern.divisionsByStep = (int)((ss + ds) / 0.02);
	johnny.walkingPattern.lateralMovement = atoi(argv[4]); 
	johnny.walkingPattern.footHeight = atoi(argv[5]); 
	johnny.walkingPattern.chestFlexion = (double)atoi(argv[6]) / 10.0;
	johnny.walkingPattern.stepsNumber = atoi(argv[7]);
	std::cout 
		<< "\nStep lenght: " << atoi(argv[1]) << " mm\n"
		<< "Single support duration: " << atoi(argv[2]) << " ms\n"
		<< "Double support duration: " << atoi(argv[3]) << " ms \n"
		<< "Lateral movement: " << atoi(argv[4]) << " mm\n"
		<< "Foot Height: " << atoi(argv[5]) << " mm\n"
		<< "Chest Flexion: " << (double)atoi(argv[6]) / 10.0 << " °\n"
		<< "Number of steps: " << atoi(argv[7]) << "\n\n";
	johnny.wait(3.0);
	johnny.walk();

    return 1;
}
