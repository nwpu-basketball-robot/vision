#include "detector.h"


detector::~detector()
{

}
detector::detector(string namex)
{
	name=namex;
}
void detector::setimg(Mat& imgx)
{
	//imgx.copyTo(testimg);
	testimg=imgx;
	resize(testimg, testimg, Size(imgsize, imgsize * testimg.rows / testimg.cols), 0, 0, INTER_LINEAR);
}

detector::detector(Mat& imgx,string namex)
{
	imgx.copyTo(testimg);
	name=namex;
}


void detector::mainfunc(vector<Rect>& minRectx)
{
	minRectx.clear();

	if(testimg.empty())
	return ;

	Mat imgg;
	Mat imggd;
	Mat img2;
	testimg.copyTo(img2);
	Mat cop=Mat::zeros( testimg.size(), CV_8UC3 );
	rcd.mainfunc1(testimg,imggd);

	max_profile maxpd(imggd,"maxpd",dl);
	maxpd.getdrawing(); 

	get_rects grsd(maxpd.drawing,"grsd");
	grsd.mainfunc(); 

	rcc.mainfunc(testimg, imgg); 

	Mat element = getStructuringElement(MORPH_RECT, Size(erodesize, erodesize));  
	Mat imggout;
	erode(imgg, imggout, element);

	////imshow(name+"inrange1",rcc.dst);
	////imshow(name+"inrange2",rcc.dst_);

	////imshow(name+"delete",rcd.dst);
	for(unsigned int i = 0; i< grsd.minRect.size(); i++ )
	{
		Scalar color=Scalar(0,0,0);
		rectangle(imggout,grsd.minRect[i],color, -1, 4);
	}
	////imshow(name+"imggout",imggout);

	max_profile maxp(imggout,"maxp",dl);
	maxp.getdrawing();

	get_rects grs(maxp.drawing,"grs");
	grs.mainfunc();

	for(unsigned int i = 0; i< grs.minRect.size(); i++ )
	{
		Scalar color=Scalar(0,255,0);
		
		rectangle(cop,grs.minRect[i],color, -1, 4);
	}

	get_rects grs2(cop,"grs2");
	grs2.mainfunc();
	for(unsigned int i = 0; i< grs2.minRect.size(); i++ )
	{
		if(abs(grs2.minRect[i].height*grs2.minRect[i].width)>lthresh*lthresh
			&& abs(grs2.minRect[i].height/grs2.minRect[i].width)<khw
			&& abs(grs2.minRect[i].width/grs2.minRect[i].height)<khw
			)
		{
			Scalar color=Scalar(0,255,0);
			string words;
			
			color=Scalar(255,0,0);
			rectangle(img2,grs2.minRect[i],color, 1, 1);
			words="item";
			putText(img2,words,grs2.minRect[i].tl(),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),2,4);
			minRectx.push_back(grs2.minRect[i]);
		}
	}
	////imshow(name+"cop",cop);
	//imshow(name+"result",img2);
	
	return;

}