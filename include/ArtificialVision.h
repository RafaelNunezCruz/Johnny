/*
 *  ArtificialVision.h
 *
 *   Author: Rafael Nuñez
 *
 */

#ifndef _ARTIFICIAL_VISION_H_
#define _ARTIFICIAL_VISION_H_

	#include <stdio.h>
	#include <iostream>
	#include "opencv2/core/core.hpp"
	#include "opencv2/features2d/features2d.hpp"
	#include "opencv2/highgui/highgui.hpp"
	#include "opencv2/imgproc/imgproc.hpp"

	using namespace cv;

	class ArtificialVision{
		char windowName[20];
		CvCapture* cameraCapture;
	public:
		int segmentedColor;
		ArtificialVision();
		void startRecognition();
		void stopRecognition();
		bool getcircleCoordinates(int circleCoordinates[]);
	};

#endif
