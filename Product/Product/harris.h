#pragma once
#ifndef HARRIS_H
#define HARRIS_H

#include<iostream>
#include "cv.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "highgui.h"


using namespace cv;
using namespace std;


class Harris
{
public:
	Harris();
	Harris(char*const image_name);
	virtual ~Harris();
protected:
private:
	IplImage *initImage;
	IplImage *grayImage_8U;
	IplImage *grayImage_64F;
	IplImage *Ix, *Iy, *Ixx, *Iyy, *Ixy;
	IplImage *A, *B, *C;
	IplImage *harris_image;
	double *gauss_weight;
	IplImage* Mul(IplImage* M1, IplImage*M2);
	IplImage* MW(IplImage *M);
	IplImage* harris_conner();
	IplImage* nonMaxValueSuppression();
};


#endif // HARRIS_H
