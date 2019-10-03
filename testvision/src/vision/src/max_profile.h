#ifndef MAX_PROFILE_H
#define MAX_PROFILE_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

class max_profile
{
public:
	Mat src;
	Mat src_;
	Mat src_gray;
	Mat drawing;
	Mat drawing2;
	string name;

	Point cpt;

	int thresh = 249;
	int threshold1=20;//95//120
	int threshold2=160;//100
	int max_thresh = 255;
	int dl;

	double x,y;
	vector<vector<Point> > contours;
	void getprofile(vector<vector<Point> >& contours);
	Rect key_rect;
	Point2f gcenter(vector<vector<Point> >&contours1,int t,Mat& src);
	max_profile(Mat& imgx,string namex,int dlx);
	void mainfunc();
	void getdrawing();

	~max_profile();

};
#endif // MAX_PROFILE_H
