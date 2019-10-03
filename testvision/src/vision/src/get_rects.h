#ifndef GET_RECTS_H
#define GET_RECTS_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

class get_rects
{
public:
	Mat src;
	Mat src_gray;
	int thresh = 20;
	int max_thresh = 255;
	
	vector<Rect> minRect;
	int maxi;

	string name;

	get_rects(Mat& imgx,string namex);
	void mainfunc();



	~get_rects();
};

#endif