#include "get_rects.h"

get_rects::~get_rects()
{

}

get_rects::get_rects(Mat& imgx,string namex)
{
	imgx.copyTo(src);
	cvtColor( src, src_gray, CV_BGR2GRAY );
	blur( src_gray, src_gray, Size(3,3) );
	name=namex;
}

void get_rects::mainfunc()
{
	Mat canny_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Canny( src_gray, canny_output, thresh, thresh*3, 3 );
	findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
	Mat drawing2 = Mat::zeros( canny_output.size(), CV_8UC3 );
	minRect.resize( contours.size() );
	for(unsigned int i = 0; i < contours.size(); i++ )
	{
		minRect[i] = boundingRect( Mat(contours[i]) );
	}

	double maxs3=0;//最大面积
	int t3=1;//记录拥有最大矩形的轮廓号
	for(unsigned int i = 0; i< minRect.size(); i++ )
	{
	 		
		if(abs(maxs3)<abs(minRect[i].height*minRect[i].width)&&
		abs(minRect[i].height*minRect[i].width)<src.cols*src.rows*0.95)//这一行避免t记录图片框
		{
			maxs3=minRect[i].height*minRect[i].width;
			t3=i;
		}

	}

	if(t3<minRect.size() && minRect[t3].height*minRect[t3].width > 200)
	{
		
		Scalar color=Scalar(0,255,0);
		
		rectangle(drawing2,minRect[t3],color, 3, 8);
		maxi=t3;
	}

	for(unsigned int i = 0; i< contours.size(); i++ )
	{
		RNG rng(12345);

		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		rectangle(drawing2,minRect[i],color, 3, 8);
	}

	////namedWindow( name+"drawing2", CV_WINDOW_AUTOSIZE );
	////imshow( name+"drawing2", drawing2 );

}