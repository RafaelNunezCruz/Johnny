#include "Johnny.h" 

int main() {
	std::cout << "\n=== Johnny Vision Demo ===\n\n";
	Johnny johnny;
	johnny.homePosition(1.0);

	double totalTime = 20.0, incTime = 0.1, ti, dt;
	int totalcycles = (int)(totalTime / incTime);
	int circleCoordinates[2];

	johnny.artificialVision.startRecognition();
	
	// 0: white
	// 1: blue
	// 2: green
	// 3: red
	// 4: yellow	
	
	johnny.artificialVision.segmentedColor = 4;
	
	for(int k = 0; k < totalcycles ; k++){
		ti = johnny.getTime();
		bool success = johnny.artificialVision.getcircleCoordinates(circleCoordinates);
		dt = johnny.getTime() - ti;
		std::cout << "Circle segmentation takes: " << dt << " seconds" << std::endl;
		
		if(success)
			std::cout << "Circle coordinates:" << circleCoordinates[0] << ", " << circleCoordinates[1] << std::endl;		
		else
			std::cout << "Circle not found" << std::endl;
		
		if (incTime - dt > 0.0)
			johnny.wait(incTime - dt);
		else
			std::cout << "Not enough time to wait" << std::endl;
	}
	johnny.artificialVision.stopRecognition();

    return 0;
}
