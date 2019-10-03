#include "range_cut.h"

range_cut::range_cut()
{
	cnt=0;
	cnt1=0;
}
void range_cut::has(Mat& img,int& num)
{
	num=0;
	img.convertTo(bgr, CV_32FC3, 1.0 / 255, 0);
	cvtColor(bgr, hsv, COLOR_BGR2HSV);
	inRange(hsv, Scalar(hmin, smin / float(smin_Max), vmin / float(vmin_Max)), Scalar(hmax, smax / float(smax_Max), vmax / float(vmax_Max)), mask);
	for (int r = 0; r < bgr.rows; r++)
	{
		for (int c = 0; c < bgr.cols; c++)
		{

			if (mask.at<uchar>(r, c) == 255)
			{
				
				num++;
			}
		}
	}
}
void  range_cut::mainfunc(Mat& img ,Mat& imgg)
{
	dst = Mat::zeros(img.size(), CV_32FC3);
	dst_ = Mat::zeros(img.size(), CV_32FC3);
	dstfinal = Mat::zeros(img.size(), CV_32FC3);
	//int num=0;
	int numfinal=0;

	img.convertTo(bgr, CV_32FC3, 1.0 / 255, 0);
	cvtColor(bgr, hsv, COLOR_BGR2HSV);

	inRange(hsv, Scalar(hmin1, smin1 / float(smin_Max), vmin1 / float(vmin_Max)), Scalar(hmax1, smax1 / float(smax_Max), vmax1 / float(vmax_Max)), mask1);
	inRange(hsv, Scalar(hmin, smin / float(smin_Max), vmin / float(vmin_Max)), Scalar(hmax, smax / float(smax_Max), vmax / float(vmax_Max)), mask);
	for (int r = 0; r < bgr.rows; r++)
	{
		for (int c = 0; c < bgr.cols; c++)
		{
			if (mask.at<uchar>(r, c) == 255 || mask1.at<uchar>(r, c) == 255)
			{
				dstfinal.at<Vec3f>(r, c) = bgr.at<Vec3f>(r, c);
				numfinal++;
			}

			if (mask1.at<uchar>(r, c) == 255)
			{
				dst_.at<Vec3f>(r, c) = bgr.at<Vec3f>(r, c);
				//num++;
			}

			if (mask.at<uchar>(r, c) == 255)
			{
				dst.at<Vec3f>(r, c) = bgr.at<Vec3f>(r, c);
				//num++;
			}
		}
	}
	cnt=numfinal;
	
	dst.convertTo(dst, CV_8UC3, 255.0, 0);
	dst_.convertTo(dst_, CV_8UC3, 255.0, 0);
	dstfinal.convertTo(dstfinal, CV_8UC3, 255.0, 0);
	
	dstfinal.copyTo(imgg);
	
}

void  range_cut::mainfunc1(Mat& img ,Mat& imgg)
{
	img.convertTo(bgr, CV_32FC3, 1.0 / 255, 0);
	cvtColor(bgr, hsv, COLOR_BGR2HSV);


	dst = Mat::zeros(img.size(), CV_32FC3);
	Mat mask;

	inRange(hsv, Scalar(hmin, smin / float(smin_Max), vmin / float(vmin_Max)), Scalar(hmax, smax / float(smax_Max), vmax / float(vmax_Max)), mask);
	for (int r = 0; r < bgr.rows; r++)
	{
		for (int c = 0; c < bgr.cols; c++)
		{
			if (mask.at<uchar>(r, c) == 255)
			{
				dst.at<Vec3f>(r, c) = bgr.at<Vec3f>(r, c);
			}
		}
	}
	//namedWindow("inrange",CV_WINDOW_NORMAL);
	//imshow("inrange", dst);
	dst.convertTo(dst, CV_8UC3, 255.0, 0);
	
	dst.copyTo(imgg);
	
}

void range_cut::threshset(int hi,int hm,int si,int sm,int vi,int vm)
{
	hmin=hi;
	hmax=hm;
	smin=si;
	smax=sm;
	vmin=vi;
	vmax=vm;

}
void range_cut::threshset_(int hi,int hm,int si,int sm,int vi,int vm)
{
	hmin1=hi;
	hmax1=hm;
	smin1=si;
	smax1=sm;
	vmin1=vi;
	vmax1=vm;
}

