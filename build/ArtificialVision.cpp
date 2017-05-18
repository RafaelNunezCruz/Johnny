/*
 *  ArtificialVision.cpp
 *
 *   Author: Rafael Nuñez
 *
 */

#include "ArtificialVision.h"


ArtificialVision::ArtificialVision(){
	strcpy(windowName,"Segmented");
	segmentedColor = 3;
}

void ArtificialVision::startRecognition(){
	namedWindow( windowName, CV_WINDOW_AUTOSIZE );
	cameraCapture = cvCreateCameraCapture(-1);
	cvSetCaptureProperty(cameraCapture,CV_CAP_PROP_FRAME_WIDTH,320);
	cvSetCaptureProperty(cameraCapture,CV_CAP_PROP_FRAME_HEIGHT,240);
	Mat src = cvQueryFrame(cameraCapture);		
	imshow(windowName,src);
	std::cout << "Window opened" << std::endl;
	cvWaitKey(200);
}

void ArtificialVision::stopRecognition(){
	cvReleaseCapture(&cameraCapture);
	cvDestroyWindow(windowName);
	std::cout << "Window closed" << std::endl;
}

bool ArtificialVision::getcircleCoordinates( int circleCoordinates[]){	
	bool success = false;
	Mat src,img_HSV,segmentedImage;	
	vector<Vec3f> circles;
	Scalar color_i,color_f;

	circleCoordinates[0] = -1;
	circleCoordinates[1] = -1;
	
	if(!cameraCapture){
		std::cout << "Could not initialize capturing" << std::endl;
	}
	else{
		switch(segmentedColor){
			case 0: color_i = Scalar(0  ,0  ,185); color_f = Scalar(255,75 ,255); break;//white
			case 1: color_i = Scalar( 90,55 ,95 ); color_f = Scalar(125,215,205); break;//blue
			case 2: color_i = Scalar(50 ,120,120); color_f = Scalar(90 ,255,255); break;//green
			case 3: color_i = Scalar(0  ,140,180); color_f = Scalar(35 ,240,250); break;//red
			case 4: color_i = Scalar(25 ,60 ,140); color_f = Scalar(40 ,120,250); break;//yellow	
		}

		src = cvQueryFrame(cameraCapture);
	       
		cvtColor(src, img_HSV, CV_BGR2HSV);
		inRange(img_HSV,color_i,color_f,segmentedImage); 	
		GaussianBlur( segmentedImage, segmentedImage, Size(9, 9), 2, 2 );		
		HoughCircles( segmentedImage, circles, CV_HOUGH_GRADIENT, 2.05, segmentedImage.rows/8,100, 100, 0,150 );	

		if(	circles.size() > 0){
			Point center(cvRound(circles[0][0]), cvRound(circles[0][1]));
			int radius = cvRound(circles[0][2]);
			circle( src, center, radius, Scalar(100,0,0), 3, 8, 0 );
			circleCoordinates[0]= circles[0][0]-320;
			circleCoordinates[1]=-circles[0][1]+480;
			success = true;
		}
		imshow(windowName,src);
		cvWaitKey(20);
	}
	return success;
}
