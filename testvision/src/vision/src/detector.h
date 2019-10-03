#ifndef DETECTOR_H
#define DETECTOR_H

#include "get_rects.h"
#include "max_profile.h"
#include "range_cut.h"

#define imgsize 400

class detector
{
public:
	string name;
	int lthresh;
	int khw;
	int erodesize;
	int dl;
	Mat testimg;
	range_cut rcc;
	range_cut rcd;
	int model;
	detector(Mat& imgx,string namex);
	void setimg(Mat& imgx);
	detector(string namex);
	void mainfunc(vector<Rect>& minRectx);
	~detector();
	void setmode(int lthreshx,int khwx,int erodesizex,int dlx,int hminx,int hmaxx,int sminx,int smaxx,int vminx,int vmaxx,int hmin1x,int hmax1x,int smin1x,int smax1x,int vmin1x,int vmax1x,int hmind,int hmaxd,int smind,int smaxd,int vmind,int vmaxd)
	{
		lthresh=lthreshx;
		khw=khwx;
		erodesize=erodesizex;
		dl=dlx;
		rcc.hmin=hminx;
		rcc.hmax=hmaxx;
		rcc.smin=sminx;
		rcc.smax=smaxx;
		rcc.vmin=vminx;
		rcc.vmax=vmaxx;
		rcc.hmin1=hmin1x;
		rcc.hmax1=hmax1x;
		rcc.smin1=smin1x;
		rcc.smax1=smax1x;
		rcc.vmin1=vmin1x;
		rcc.vmax1=vmax1x;
		rcd.hmin=hmind;
		rcd.hmax=hmaxd;
		rcd.smin=smind;
		rcd.smax=smaxd;
		rcd.vmin=vmind;
		rcd.vmax=vmaxd;
	}
}
;

#endif
