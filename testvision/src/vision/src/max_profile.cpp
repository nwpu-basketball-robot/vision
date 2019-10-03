#include "max_profile.h"

void max_profile::mainfunc()
{
	int ats = cv::getTickCount();
	blur( src, src, Size(3,3) );
	cvtColor( src, src_gray, CV_BGR2GRAY );
	Mat threshold_output1;
	vector<vector<Point> > contours1;
	vector<Vec4i> hierarchy;
	cv::Canny(src_gray,threshold_output1,threshold1,threshold2,3, false);
	findContours( threshold_output1, contours1, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	getprofile(contours1);
	cout<<x<<","<<y<<endl;

	ats = cv::getTickCount() - ats;
	 
	cout<< "maxpf using time:" << ats / cv::getTickFrequency() * 1000 << endl;
}

max_profile::max_profile(Mat& imgx,string namex,int dlx)
{
	imgx.copyTo(src);
	name = namex;
	dl=dlx;
}


Point2f max_profile::gcenter( vector<vector<Point> >&contours1 ,int t,Mat& src)
{
	long long sumx=0,sumy=0;

	if(t<contours1.size())
	{	
		int radius=1;
		cpt.x = key_rect.x + cvRound(key_rect.width / 2.0);
		cpt.y = key_rect.y + cvRound(key_rect.height / 2.0);

		circle(src, cpt, radius, Scalar(255, 0, 0), 5, 8, 0);
		return Point2f(0,0);
	}
	else return Point2f(0,0);
}


void max_profile::getprofile(vector<vector<Point> >& contours)
{
	drawing = Mat::zeros( src.size(), CV_8UC3 );
	src.copyTo(src_);
	vector<Rect> minRect1( contours.size() );
	for(unsigned int i = 0; i < contours.size(); i++ )
	{
		minRect1[i] = boundingRect( Mat(contours[i]) );
	}

	double maxs=0;//最大面积
	int t=1;//记录拥有最大矩形的轮廓号
	 
	for(unsigned int i = 0; i< contours.size(); i++ )
	{
		Scalar color=Scalar(0,255,0);
		drawContours( drawing, contours, i, color,8 , 8, vector<Vec4i>(), 0, Point() );//15
		if(abs(maxs)<abs(minRect1[i].height*minRect1[i].width)&&
			abs(minRect1[i].height*minRect1[i].width)<src.cols*src.rows*0.95)//这一行避免t记录图片框
		{
			maxs=minRect1[i].height*minRect1[i].width;
			t=i;
		}
	}

	///namedWindow(name+ "drawing", CV_WINDOW_NORMAL );

	///imshow( name+"drawing", drawing );
	Mat threshold_output3;
	vector<vector<Point> > contours3;
	vector<Vec4i> hierarchy;

	blur( drawing, drawing, Size(3,3) );
	cv::Canny(drawing,threshold_output3,threshold1,threshold2,3, false);
	findContours( threshold_output3, contours3, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	drawing2 = Mat::zeros( threshold_output3.size(), CV_8UC3 );


	vector<Rect> minRect3( contours3.size() );
	for(unsigned int i = 0; i < contours3.size(); i++ )
	{
		minRect3[i] = boundingRect( Mat(contours3[i]) );
	}
	double maxs3=0;//最大面积
	int t3=1;//记录拥有最大矩形的轮廓号
	for(unsigned int i = 0; i< contours3.size(); i++ )
	{
		
		if(abs(maxs3)<abs(minRect3[i].height*minRect3[i].width)&&
			abs(minRect3[i].height*minRect3[i].width)<src.cols*src.rows*0.95)//这一行避免t记录图片框
		{
			maxs3=minRect3[i].height*minRect3[i].width;
			t3=i;
		}
	}

	if(t3<contours3.size())
	{
		 
		Scalar color=Scalar(0,255,0);
		drawContours( src_, contours3, t3, color, 1, 8, vector<Vec4i>(), 0, Point() );
	}
	RNG rng(12345);
	for(unsigned int i = 0; i< contours3.size(); i++ )
	{
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours( drawing2, contours3, i, color, 3, 8, vector<Vec4i>(), 0, Point() );
			
			
	}

	if(t3<contours3.size())
	{
		rectangle(src_,minRect3[t3],Scalar(0,255,0), 1, 8);
		key_rect=minRect3[t3];
		
	}


		
	Point2f zhongxin(gcenter(contours3,t3,src_));
	x=zhongxin.x;
	y=zhongxin.y;
	///namedWindow(name+"drawing2",CV_WINDOW_NORMAL);
	///imshow(name+"drawing2",drawing2);

	namedWindow(name+"src",CV_WINDOW_NORMAL);
	imshow(name+"src", src_ );
}
void max_profile::getdrawing()
{
	drawing = Mat::zeros( src.size(), CV_8UC3 );
	cvtColor( src, src_gray, CV_BGR2GRAY );
	
	Mat threshold_output1;
	vector<vector<Point> > contours1;
	vector<Vec4i> hierarchy;
	cv::Canny(src_gray,threshold_output1,threshold1,threshold2,3, false);
	findContours( threshold_output1, contours1, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	
	
	src.copyTo(src_);
	
	vector<Rect> minRect1( contours1.size() );
	for(unsigned int i = 0; i < contours1.size(); i++ )
	{
		minRect1[i] = boundingRect( Mat(contours1[i]) );
	}

	
	for(unsigned int i = 0; i< contours1.size(); i++ )
	{
		Scalar color=Scalar(0,255,0);
		drawContours( drawing, contours1, i, color,dl, 8, vector<Vec4i>(), 0, Point() );
	
	}
	////namedWindow(name+"drawing",CV_WINDOW_NORMAL);
	////imshow(name+"drawing", drawing );

}

max_profile::~max_profile()
{

}
