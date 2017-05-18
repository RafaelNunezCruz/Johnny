/*
 *  JohnnyInfo.h
 *
 *   Author: Rafael Nuñez
 *
 */

#ifndef _JOHNNY_INFO_H_
#define _JOHNNY_INFO_H_
	
	#include <math.h>
	#include <iostream>
	#include "FilteredSignal.h"

	namespace Joints{
		enum {
			R_ANKLE_EVERSION, // Joint 0
			R_ANKLE_FLEXION, // Joint 1
			R_KNEE_FLEXION, // Joint 2
			R_HIP_ABDUCTION, // Joint 3
			R_HIP_FLEXION, // Joint 4
			R_HIP_ROTATION, // Joint 5
			L_HIP_ROTATION, // Joint 6
			L_HIP_FLEXION, // Joint 7
			L_HIP_ABDUCTION, // Joint 8
			L_KNEE_FLEXION, // Joint 9
			L_ANKLE_FLEXION, // Joint 10
			L_ANKLE_EVERSION, // Joint 11
			COLUMN_FLEXION, // Joint 12
			COLUMN_ABDUCTION, // Joint 13
			R_SHOULDER_FLEXION, // Joint 14
			R_SHOULDER_ABDUCTION, // Joint 15
			R_SHOULDER_ROTATION, // Joint 16
			R_ELBOW, // Joint 17
			R_HAND, // Joint 18
			L_SHOULDER_FLEXION, // Joint 19
			L_SHOULDER_ABDUCTION, // Joint 20
			L_SHOULDER_ROTATION, // Joint 21
			L_ELBOW, // Joint 22
			L_HAND, // Joint 23
			HEAD_ROTATION, // Joint 24
			HEAD_FLEXION, // Joint 25
			NUMBER_OF_JOINTS // 26
		};
	};

	namespace Dimensions{		
		const double THIGH_LENGHT = 159.0;
		const double SHANK_LENGHT = 156.0;
		const double HIP_WIDTH = 130.0;
		const double UPPER_ARM_LENGHT = 130.0;
		const double FOREARM_LENGHT = 180.0;
		const double ANKLE_HEIGHT = 59.0;
		const double R_HIP_CM_DX = 65.0; //HIP_WIDTH / 2
		const double R_HIP_CM_DY = 45.0;
		const double R_HIP_CM_DZ = -20.0;
		const double CM_L_HIP_DX = 65.0; //R_HIP_CM_DX
		const double CM_L_HIP_DY = -45.0; //-R_HIP_CM_DY
		const double CM_L_HIP_DZ = 20.0; //-R_HIP_CM_DZ
		const double COLUMN_INF = 83.0;
		const double COLUMN_MED = 27.0;
		const double COLUMN_SUP = 180.0;
		const double R_SHOULDER_CHEST_DX = 130.0;
		const double CHEST_L_SHOULDER_DX = 130.0; //R_SHOULDER_CHEST_DX
		const double NECK_LENGHT = 60.0;
		const double HEAD_LENGHT = 30.0;
	};

	namespace Pose{
		enum {
			TX, TY, TZ,
			RX,	RY,	RZ,
			SIZE
		};
	};

	namespace Home{
		using namespace Dimensions;
		using namespace Pose;
		const double centerMassHome[] = {R_HIP_CM_DX, THIGH_LENGHT + SHANK_LENGHT + R_HIP_CM_DY, R_HIP_CM_DZ, 0.0, 0.0, 0.0};
		const double leftFootHome[] = {HIP_WIDTH, 0.0, 0.0, 0.0, 0.0, 0.0};
		const double chestHome[] = {leftFootHome[TX] / 2.0, centerMassHome[TY] - R_HIP_CM_DY + COLUMN_INF + COLUMN_MED + COLUMN_SUP, 0.0, 0.0, 0.0, 0.0};
		const double headHome[]  = {chestHome[TX], chestHome[TY] + NECK_LENGHT + HEAD_LENGHT, 0.0, 0.0, 0.0, 0.0};
		const double rightHandHome[] = {chestHome[TX] - R_SHOULDER_CHEST_DX, chestHome[TY] - UPPER_ARM_LENGHT - FOREARM_LENGHT, 0.0, 0.0, 0.0, 0.0};
		const double leftHandHome[]  = {chestHome[TX] + R_SHOULDER_CHEST_DX, chestHome[TY] - UPPER_ARM_LENGHT - FOREARM_LENGHT, 0.0, 0.0, 0.0, 0.0};	
	};

	namespace Actuators{
//		Actuator ID:                    100  101  102  103  104  105  106  107  108  109  110  111  112  113  114  115  116  117  118  119  120  121  122  123  124  125
		const int MIN_ANGLE_WORD[]  = {1840,1810,2020,1840,1700,1350,1760,1700,1840,1656,1840,1840,1890,1900,1500,1500, 218,1000,1700,1500,1170,1858,1652,1400,1800,1000};
		const int HOME_ANGLE_WORD[] = {2042,2031,2048,2046,2244,1792,2116,2064,2033,2047,2065,2063,2306,2072,2047,1740,1258,1009,1905,2051,2542,2820,3049,1628,2099,1306};
		const int MAX_ANGLE_WORD[]  = {2240,2240,2423,2340,2420,2250,2760,2420,2300,2060,2500,2240,2390,2200,2600,3100,2218,2417,1920,2600,2770,3858,3165,2148,2400,1800};
		const double P_GAIN[]       = {15.0,15.0,15.5,20.5,15.5,12.0,12.0,15.5,20.5,15.5,15.0,15.0,10.0,10.0, 2.0, 1.0, 2.0, 2.0, 2.5, 2.0, 1.0, 2.0, 2.0, 2.5, 2.5, 2.5}; // P_GAIN < 20.672
		const double I_GAIN[]       = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // I_GAIN < 8.075 
		const double D_GAIN[]       = { 5.0, 9.5, 5.5, 5.0, 5.5, 5.0, 5.0, 5.5, 5.0, 5.5, 9.5, 5.0, 5.0, 5.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // D_GAIN < 66.151
		const int DIRECTION[]       = {  -1,  -1,   1,   1,  -1,  -1,   1,   1,   1,  -1,   1,  -1,   1,   1,  -1,  -1,   1,  -1,  -1,   1,  -1,   1,   1,   1,   1,   1};
		const int INDEX_LEFT[]      = {  11,  10,   9,   8,   7,   6,   5,   4,   3,   2,   1,   0,  12,  13,  19,  20,  21,  22,  23,  14,  15,  16,  17,  18,  24,  25};

		const double RATIO_VELOCITY2WORD = 1.462;
		const double RATIO_P_GAIN2WORD = 12.2871;
		const double RATIO_I_GAIN2WORD = 31.4550;
		const double RATIO_D_GAIN2WORD =  3.8397;
	};
	
	namespace Sensors{
		const std::string RIGHT_PORT_NAME = "/dev/serial/by-path/pci-0000:00:1d.0-usb-0:1:1.0";
		const std::string LEFT_PORT_NAME  = "/dev/serial/by-path/pci-0000:00:1d.0-usb-0:2:1.0";
		const char STOP_TRANSMISSION[]  = {170, 0, 50, 3,  0, 4, 0, 0, 227};
		const char START_TRANSMISSION[] = {170, 0, 50, 3, 10, 4, 0, 0, 237};
		const int SAMPLE_SIZE = 22;
		const int TIMEOUT = 500;
		const int DATA_OFFSET_RIGHT[] = { 21, 66, 232, -179,  36,  0};
		const int DATA_OFFSET_LEFT[]  = { -9,  9, 238,  -37, -63, 22};		
		const double CHEST_INCLINATION_OFFSET[]  = {0.0, -2.5, -3.5};
		enum {FX, FY, FZ, MX, MY, MZ, OPTOFORCE_SIZE} ;
		const double VALUE2COUNTS_RATIO_RIGHT[] = {33.74, 34.00, 4.05, 603.75, 571.45, 865.70}; //Serial Number: ICE027
		const double VALUE2COUNTS_RATIO_LEFT[]  = {35.88, 38.55, 4.08, 608.47, 609.74, 936.82}; //Serial Number: ICE041
		static const int RIGHT_FOOT = 1;
		static const int LEFT_FOOT = -1;
	};
	
	class SensorsData{
	public:
		FilteredSignal chestInclination;
		FilteredSignal chestAngularVelocity;
		FilteredSignal zmp;
	};
	
	class Control{
	public:
		double pGain, iGain, dGain;
	};
#endif
