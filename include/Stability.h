/*
 *  Stability.h
 *
 *   Author: Rafael Nuñez
 *
 */

#ifndef _STABILITY_H_
#define _STABILITY_H_

	#include "FilteredSignal.h"
	#include "JohnnyInfo.h"

	class Stability{
		static double saturation(double value, double max, double min);
	public:
		static const double MIN_R_HIP_ABDUCTION = 2.0;
		static const double ANKLE_EVERSION_COMPENSATION = 1.15;
		static const double SATURATION_ANGLE = 7.0;
		static void gravityCompensation(double jointsAngles[], int side, double columnInclination);
		static void zmpControl(double jointsAngles[], FilteredSignal zmp, FilteredSignal zmpReference, Control ctrlX, Control ctrlZ);
	};

#endif
