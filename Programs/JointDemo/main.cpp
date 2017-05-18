#include "Johnny.h" 

int main(int argc,char *argv[]) {
	std::cout << "\n=== Johnny Joint Demo ===\n\n";
	Johnny johnny;
	
	if (argc != 3) {
		std::cout 
		<< "\nTo run this program you should define:\n"
		<< "Index (0-25)\n"
		<< "Angle (deg)\n"
		<< "Hint: ./JointDemo 24 45\n\n";
		johnny.homePosition(1.0);
		return 0;		
	}

	int index = atoi(argv[1]);
	int angle = atoi(argv[2]);

	johnny.actuators.moveJoint(index, angle, 1.0);
	johnny.wait(1.0);

    return 0;
}
