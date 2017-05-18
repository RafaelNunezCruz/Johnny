#include "Johnny.h" 

Johnny johnny;	

const int MAX_CMD_LEN = 10;
const int MAX_NUM_PARAM = 2;
const int MAX_PARAM_LEN = 20;
const int MAX_INPUT_LEN = MAX_CMD_LEN + 1 + MAX_NUM_PARAM * (MAX_PARAM_LEN + 1);

bool fileExists(const char *fileName);
bool loadFile(const char *fileName, int goalPose[]);
bool createFile(const char *fileName, int goalPose[]);
bool getCommand(char cmd[], int &num_param, char param[][MAX_PARAM_LEN]);
void delta(int num_param, char param[][MAX_PARAM_LEN], int goalPose[]);
void save(int goalPose[], const char *fileName);
void help(void);

int main(int argc,char *argv[]) 
{
	int goalPose[NUMBER_OF_JOINTS] = {};
	const char *fileName;	
	
	std::cout << "\n==== Johnny Home Calibration ====\n\n";
	
	if (argc != 2) 
	{
		std::cout 
		<< "\nYou did not define a valid pose file\n"
		<< "Hint: ./HomeCalibration file.txt\n\n"
		<< "If the file already exist you will edit it\n"
		<< "If the file does not exist it will be created using current home pose\n\n";
		return 0;		
	}
	else 
	{
		fileName = argv[1];	
		bool success;
		
		if(fileExists(fileName))
			success = loadFile(fileName, goalPose);
		else
		{
			johnny.homePosition(1.0);
			johnny.actuators.readAngles();
			success = createFile(fileName, goalPose);
		}
		if(!success)
			return 0;			
	}
	

	std::cout << "Type \"help\" to see available commands\n\n";
	
	char cmd[MAX_CMD_LEN];
	char param[MAX_NUM_PARAM][MAX_PARAM_LEN];
	int num_param;
	
	while(1)
	{
		std::cout << "> ";
		if(getCommand(cmd, num_param, param))
		{
			std::cerr << "\nError reading command\n\n";
			help();
			continue;		
		}
		else if(strcmp(cmd, "delta") == 0)		
			delta(num_param, param, goalPose);		
		else if(strcmp(cmd, "save") == 0)		
			save(goalPose, fileName);	
		else if(strcmp(cmd, "help") == 0)		
			help();			
		else if(strcmp(cmd, "exit") == 0)
			break;
		else
		{
			std::cout << "\nUnknow command\n";
			help();
		}
	}
	
	return 0;
}

bool fileExists(const char *fileName)
{
    ifstream infile(fileName);
    return infile.good();
}

bool loadFile(const char *fileName, int goalPose[])
{
	std::ifstream file(fileName);
	if(file.is_open())
	{
		std::cout << "Loading file: "<< fileName << "\n\n";
		int position;
		double goalAngle[NUMBER_OF_JOINTS] = {};
		for(int joint = 0; joint < NUMBER_OF_JOINTS; joint++)
		{
			file >> position;
			goalPose[joint] = position;
			std::cout << joint << ":\t" << goalPose[joint] << "\n";
			goalAngle[joint] = ActuatorsManipulation::word2Angle(joint, goalPose[joint]);
		}
		johnny.actuators.moveAllJoints(goalAngle, 1.0);
		return true;
	}
	else
	{
		std::cerr << "Can not open the file: " << fileName << " \n\n";	
		return false;			
	}
	file.close();
}

bool createFile(const char *fileName, int goalPose[])
{
	std::ofstream file(fileName);
	if(file.is_open())
	{
		std::cout << "Creating file: "<< fileName << "\n\n";
		for(int joint = 0; joint < NUMBER_OF_JOINTS; joint++)
		{
			goalPose[joint] = Actuators::HOME_ANGLE_WORD[joint];
			file << goalPose[joint] << "\n"; 
			std::cout << joint << ":\t" << goalPose[joint] << "\n";	
		}
		return true;
	}
	else
	{
		std::cerr << "Can not create the file: " << fileName << " \n\n";	
		return false;			
	}
	file.close();
}

bool getCommand(char cmd[], int &num_param, char param[][MAX_PARAM_LEN])
{
	char input[MAX_INPUT_LEN];
	gets(input);
	fflush(stdin);
	
	int input_len = strlen(input);
	if(input_len == 0)
		return true;

	char *token;
	token = strtok( input, " " );
	if(token == 0)
		return true;

	strcpy( cmd, token );
	token = strtok( 0, " " );
	num_param = 0;
	while(token != 0)
	{
		strcpy( param[num_param++], token );
		token = strtok( 0, " " );
	}
	return false;
}

void delta(int num_param, char param[][MAX_PARAM_LEN], int goalPose[])
{
	if(num_param != 2)
		std::cerr 	<< "\nYou must define the actuator id and delta angle in degrees\n"
					<< "Hint: \n  "
					<< "  > delta 0 10 (add 10 degrees to the current position of actuator 0)\n\n";
	else
	{
		int joint = atoi(param[0]);
		double delta = atof(param[1]);
		std::cout << "\nIncreasing " << delta << " degrees on actuator " << joint << "\n\n";
		double angle = ActuatorsManipulation::word2Angle(joint, goalPose[joint]) + delta; 
		johnny.actuators.moveJoint(joint, angle, 1.0);
		johnny.wait(1.0);
		goalPose[joint] = ActuatorsManipulation::angle2Word(joint, angle); 
	}
}

void save(int goalPose[], const char *fileName)
{	
	std::cout << "\nSaving file:" << fileName << "\n\n";
	std::ofstream file(fileName);
	if(file.is_open())
	{
		for(int joint = 0; joint < NUMBER_OF_JOINTS; joint++)
		{
			std::cout << joint << " :\t" << goalPose[joint] << "\n";
			file << goalPose[joint] << "\n";
		
		} 
		std::cout << "\nFile: " << fileName << " saved \n\n";	
	}
	else
		std::cerr << "Can not open or create the file: " << fileName << " \n\n";
	file.close();	
}

void help(void)
{
	std::cout 	<< "\nAvailable commands:\n\n"
				<< "  > delta id angle\n"
				<< "  > save\n"
				<< "  > help\n"
				<< "  > exit\n\n";
}
