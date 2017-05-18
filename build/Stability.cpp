/*
 *  Stability.cpp
 *
 *   Author: Rafael Nuñez
 *
 */

#include "Stability.h"

void Stability::gravityCompensation(double jointsAngles[], int side, double columnInclination)
{
	if(jointsAngles[Joints::R_HIP_ABDUCTION] < MIN_R_HIP_ABDUCTION)
		jointsAngles[Joints::R_HIP_ABDUCTION] = MIN_R_HIP_ABDUCTION;
		
	if(jointsAngles[Joints::L_HIP_ABDUCTION] > -MIN_R_HIP_ABDUCTION)
		jointsAngles[Joints::L_HIP_ABDUCTION] = - MIN_R_HIP_ABDUCTION;

	jointsAngles[Joints::COLUMN_FLEXION] = columnInclination;
	
	//jointsAngles[Joints::L_ELBOW] = -100.0;
	//jointsAngles[Joints::L_HAND] =  20.0;
}

void Stability::zmpControl(double jointsAngles[], FilteredSignal zmp, FilteredSignal zmpReference, Control ctrlX, Control ctrlZ){

	jointsAngles[Joints::COLUMN_ABDUCTION] = saturation(
											jointsAngles[Joints::COLUMN_ABDUCTION] 
											- ctrlX.pGain * ( zmp.position[Pose::TX] - zmpReference.position[Pose::TX]  ) 
											- ctrlX.iGain * ( zmp.integral[Pose::TX] - zmpReference.integral[Pose::TX]  ) 
											- ctrlX.dGain * ( zmp.velocity[Pose::TX] - zmpReference.velocity[Pose::TX]	)
											, SATURATION_ANGLE, -SATURATION_ANGLE);		
											
	jointsAngles[Joints::R_SHOULDER_ABDUCTION] = saturation(
												-2.0 * jointsAngles[Joints::COLUMN_ABDUCTION]
												, 0, -2.0 * SATURATION_ANGLE);
												
	jointsAngles[Joints::L_SHOULDER_ABDUCTION] = saturation(
												-2.0 * jointsAngles[Joints::COLUMN_ABDUCTION]
												, 2.0 * SATURATION_ANGLE, 0); 		
													
	jointsAngles[Joints::COLUMN_FLEXION] =  saturation(
											jointsAngles[Joints::COLUMN_FLEXION] 
											- ctrlZ.pGain * ( zmp.position[Pose::TZ] - zmpReference.position[Pose::TZ]  ) 
											- ctrlZ.iGain * ( zmp.integral[Pose::TZ] - zmpReference.integral[Pose::TZ]  ) 
											- ctrlZ.dGain * ( zmp.velocity[Pose::TZ] - zmpReference.velocity[Pose::TZ]	)
											, SATURATION_ANGLE, -SATURATION_ANGLE);
}

double Stability::saturation(double value, double max, double min)
{
	double _value = value;
	if(value > max)
		_value = max;	
	if(value < min)
		_value = min;
	return _value;
}
