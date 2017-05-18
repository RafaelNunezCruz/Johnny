/*
 *  FilteredSignal.cpp
 *
 *   Author: Rafael Nuñez
 *
 */

#include "FilteredSignal.h"

FilteredSignal::FilteredSignal(){
	isInit = false;
	lowPassFilterGain = 0.0;
	double zero[3] = {};
	std::copy(zero, zero + 3, _position);
	std::copy(zero, zero + 3, position);
	std::copy(zero, zero + 3, integral);
	std::copy(zero, zero + 3, velocity);
}

void FilteredSignal::addSample(double sample[], double sampleTime){
	if(isInit){
		std::copy(position, position + 3, _position);			
		for(int k = 0; k < 3; k++){
			position[k] = (1 - lowPassFilterGain) * sample[k] + lowPassFilterGain * _position[k];
			integral[k] = integral[k] + sampleTime * (position[k] + _position[k]) / 2.0;
			velocity[k] = (position[k] - _position[k]) / sampleTime;
		}	
	}
	else{
		isInit = true;
		std::copy(sample, sample + 3, position);		
	}
}
