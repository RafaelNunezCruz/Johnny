/*
 *  FilteredSignal.h
 *
 *   Author: Rafael Nuñez
 *
 */

#ifndef _FILTERED_SIGNAL_H_
#define _FILTERED_SIGNAL_H_

	#include <iostream>

	class FilteredSignal{	
		bool isInit;
		double _position[3];		
	public:		
		double lowPassFilterGain;
		double position[3];
		double velocity[3];
		double integral[3];
		
		FilteredSignal();
		void addSample(double sample[], double sampleTime);	
	};

#endif
