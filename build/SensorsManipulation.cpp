#include "SensorsManipulation.h"

Robot::LinuxCM730 SensorsManipulation::linux_cm730("/dev/ttyUSB0");
Robot::CM730 SensorsManipulation::cm730(&linux_cm730);
SerialPort SensorsManipulation::serialPortRight(RIGHT_PORT_NAME);
SerialPort SensorsManipulation::serialPortLeft(LEFT_PORT_NAME);

SensorsManipulation::SensorsManipulation(){
	if(cm730.Connect() == false)
		std::cout << "Fail to connect CM-730!" << std::endl;

	sensorsData.chestInclination.lowPassFilterGain = 0.3;
	sensorsData.chestAngularVelocity.lowPassFilterGain = 0.3;
	sensorsData.zmp.lowPassFilterGain = 0.9;
	
	isOptoforcePortsOpened = false;
}

SensorsManipulation::~SensorsManipulation(){
	closeOptoforcePorts();
}

void SensorsManipulation::openOptoforcePorts(){
	if(!isOptoforcePortsOpened){
		initializeOptoforcePort(serialPortRight);
		initializeOptoforcePort(serialPortLeft);
	}
	isOptoforcePortsOpened = true;
}

void SensorsManipulation::closeOptoforcePorts(){
	if(isOptoforcePortsOpened){
		serialPortRight.Close();
		serialPortLeft.Close();
	}
	isOptoforcePortsOpened = false;
}

void SensorsManipulation::getChestInformation(double sampleTime){	
	unsigned char table[128] = {};		
	cm730.ReadTable(Robot::CM730::ID_CM, Robot::CM730::P_GYRO_Z_L, Robot::CM730::P_ACCEL_Z_H, table, 0);
		
	double acc[] = {
		RATIO_WORD2ACCELERATION * (Robot::CM730::MakeWord(table[Robot::CM730::P_ACCEL_X_L], table[ Robot::CM730::P_ACCEL_X_H]) - CENTER_VALUE),
		RATIO_WORD2ACCELERATION * (Robot::CM730::MakeWord(table[Robot::CM730::P_ACCEL_Y_L], table[ Robot::CM730::P_ACCEL_Y_H]) - CENTER_VALUE),
		RATIO_WORD2ACCELERATION * (Robot::CM730::MakeWord(table[Robot::CM730::P_ACCEL_Z_L], table[ Robot::CM730::P_ACCEL_Z_H]) - CENTER_VALUE)
	};
	
	double gyr[] = {
		RATIO_WORD2VELOCITY * (Robot::CM730::MakeWord(table[Robot::CM730::P_GYRO_X_L], table[ Robot::CM730::P_GYRO_X_H]) - CENTER_VALUE),
		RATIO_WORD2VELOCITY * (Robot::CM730::MakeWord(table[Robot::CM730::P_GYRO_Y_L], table[ Robot::CM730::P_GYRO_Y_H]) - CENTER_VALUE),
		RATIO_WORD2VELOCITY * (Robot::CM730::MakeWord(table[Robot::CM730::P_GYRO_Z_L], table[ Robot::CM730::P_GYRO_Z_H]) - CENTER_VALUE)
	};
	
	double chestInclination[] = {	0.0 - CHEST_INCLINATION_OFFSET[0],
									atan2(acc[Pose::TZ], acc[Pose::TX]) * RATIO_RADIANS2DEGREES - CHEST_INCLINATION_OFFSET[1],
									atan2(acc[Pose::TY], acc[Pose::TX]) * RATIO_RADIANS2DEGREES - CHEST_INCLINATION_OFFSET[2]};	
	
	sensorsData.chestInclination.addSample(chestInclination, sampleTime);
	
	double chestAngularVelocity[]	= {	gyr[Pose::TY],
										gyr[Pose::TX],
										gyr[Pose::TZ]};
										
	sensorsData.chestAngularVelocity.addSample(chestAngularVelocity, sampleTime);
}

void SensorsManipulation::getZmp(int side, bool singleSupport, double sampleTime){
	double zmp[3] = {}, zmpRight[3] = {}, zmpLeft[3] = {};
	if(singleSupport){
		if(side == RIGHT_FOOT){
			getParcialZmp(RIGHT_FOOT, zmpRight);
			zmp[Pose::TX] = - zmpRight[Pose::TY]  - Dimensions::HIP_WIDTH / 2.0;
			zmp[Pose::TY] = - zmpRight[Pose::TZ];
			zmp[Pose::TZ] =   zmpRight[Pose::TX];		
		}
		else{
			getParcialZmp(LEFT_FOOT, zmpLeft);
			zmp[Pose::TX] =   zmpLeft[Pose::TY]  + Dimensions::HIP_WIDTH / 2.0;
			zmp[Pose::TY] = - zmpLeft[Pose::TZ];
			zmp[Pose::TZ] = - zmpLeft[Pose::TX];	
		}
	}
	else{
		getParcialZmp(RIGHT_FOOT, zmpRight);
		getParcialZmp(LEFT_FOOT, zmpLeft);
		zmp[Pose::TX] = - zmpRight[Pose::TY] + zmpLeft[Pose::TY];
		zmp[Pose::TY] = - zmpRight[Pose::TZ] - zmpLeft[Pose::TZ];
		zmp[Pose::TZ] =   zmpRight[Pose::TX] - zmpLeft[Pose::TX];
	}

	sensorsData.zmp.addSample(zmp, sampleTime);
	
}

void SensorsManipulation::clearFootPorts(){
	clearOptoforcePort(serialPortRight);
	clearOptoforcePort(serialPortLeft);
}

void SensorsManipulation::initializeOptoforcePort(SerialPort &serialPort){
	serialPort.Open(	SerialPort::BAUD_DEFAULT, SerialPort::CHAR_SIZE_8,
						SerialPort::PARITY_NONE, SerialPort::STOP_BITS_1,
						SerialPort::FLOW_CONTROL_NONE	);	
	serialPort.Write(STOP_TRANSMISSION);
	clearOptoforcePort(serialPort);
}

void SensorsManipulation::clearOptoforcePort(SerialPort &serialPort){
	while(serialPort.IsDataAvailable())	
		serialPort.ReadByte(TIMEOUT);
}

bool SensorsManipulation::getParcialZmp(int side, double parcialZmp[]){
	int sensorData[OPTOFORCE_SIZE] = {};
	double sensorValue[OPTOFORCE_SIZE] = {};
	double normalVector[] = {0.0, 0.0, 1.0};
	bool success = false;
	
	if(side == RIGHT_FOOT){
		success = readOptoforceSensor(serialPortRight, sensorData);
		optoforceDataConversion(sensorValue, sensorData, RIGHT_FOOT);
	}
	else{
		success = readOptoforceSensor(serialPortLeft, sensorData);
		optoforceDataConversion(sensorValue, sensorData, LEFT_FOOT);
	}
	optoforceData2zmp(parcialZmp, sensorValue, normalVector);
	
	return success;
}

bool SensorsManipulation::readOptoforceSensor(SerialPort &serialPort, int data[]){
	bool success = false;

	try {
		clearOptoforcePort(serialPort);
		serialPort.Write(START_TRANSMISSION);
		
		SerialPort::DataBuffer buffer;
		serialPort.Read(buffer, SAMPLE_SIZE, TIMEOUT);
		
		if(buffer[0] == 170 && buffer[1] == 7 && buffer[2] == 8 && buffer[3] == 16){
			data[FX] = (signed short int) (buffer[ 8] << 8 | buffer[ 9]);
			data[FY] = (signed short int) (buffer[10] << 8 | buffer[11]);
			data[FZ] = (signed short int) (buffer[12] << 8 | buffer[13]);
			data[MX] = (signed short int) (buffer[14] << 8 | buffer[15]);
			data[MY] = (signed short int) (buffer[16] << 8 | buffer[17]);
			data[MZ] = (signed short int) (buffer[18] << 8 | buffer[19]);
		}
		else
			std::cout << "-";
		
		serialPort.Write(STOP_TRANSMISSION);
		success = true;
	}
	catch (std::exception &e) {
		std::cout << "~";	
	}

	return success;
}

void SensorsManipulation::optoforceDataConversion(double value[], int data[], int side){
	if(side == RIGHT_FOOT){
		for(int k = 0; k < OPTOFORCE_SIZE; k++)
			value[k] = (data[k] - DATA_OFFSET_RIGHT[k]) / VALUE2COUNTS_RATIO_RIGHT[k];
	}
	else{
		for(int k = 0; k < OPTOFORCE_SIZE; k++)
			value[k] = (data[k] - DATA_OFFSET_LEFT[k]) / VALUE2COUNTS_RATIO_LEFT[k];
	}
}

void SensorsManipulation::optoforceData2zmp(double zmp[], double sensor[], double normalVector[]){
	double forceMagnitude = sqrt(pow(sensor[FX], 2.0) + pow(sensor[FY], 2.0) + pow(sensor[FZ], 2.0));
	if(forceMagnitude > 0.0){
		zmp[Pose::TX] = 1000.0 * (normalVector[Pose::TY] * sensor[MZ] - normalVector[Pose::TZ] * sensor[MY]) / forceMagnitude;
		zmp[Pose::TY] = 1000.0 * (normalVector[Pose::TZ] * sensor[MX] - normalVector[Pose::TX] * sensor[MZ]) / forceMagnitude;
		zmp[Pose::TZ] = 1000.0 * (normalVector[Pose::TX] * sensor[MY] - normalVector[Pose::TY] * sensor[MX]) / forceMagnitude;
	}
	else{
		zmp[Pose::TX] = 0.0;
		zmp[Pose::TY] = 0.0;
		zmp[Pose::TZ] = 0.0;	
	}	
}
