#ifndef RANGE_CUT_H
#define RANGE_CUT_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;


class range_cut
{

public:
	Mat img,bgr,hsv,dst,dst_,mask,mask1;
	Mat dstfinal;
	int hmin = 0;
	int hmin1 = 0;

	int hmin_Max = 360;

	int hmax = 360;
	int hmax1 = 360;

	int hmax_Max = 360;
	//
	int smin = 0;
	int smin1 = 0;

	int smin_Max = 255;

	int smax = 255;
	int smax1 = 255;

	int smax_Max = 255;
	//
	int vmin = 0;
	int vmin1 = 0;

	int vmin_Max = 255;

	int vmax = 255;
	int vmax1 = 255;

	int vmax_Max = 255;

	int cnt;
	int cnt1;

	range_cut();
	void mainfunc(Mat& imgx,Mat& imgg);
	void mainfunc1(Mat& imgx,Mat& imgg);
	void has(Mat& img,int& num);
	void threshset(int hi,int hm,int si,int sm,int vi,int vm);
	void threshset_(int hi,int hm,int si,int sm,int vi,int vm);
};
#endif //RANGE_CUT_H

