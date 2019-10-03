#include "basketball_msgs/ballElem.h"
#include "basketball_msgs/cylinderElem.h"
#include "basketball_msgs/visionDate.h"
#include "basketball_msgs/balls.h"
#include "basketball_msgs/cylinders.h"

#include "darknet.h"
#include "improcess.h"
#include "network.h"
#include "ros/ros.h"
#include "hokuyo_node.h"

#include <thread>
#include <opencv2/opencv.hpp>

#include "get_rects.h"
#include "max_profile.h"
#include "range_cut.h"
#include "detector.h"

#include "usb_capture_with_thread.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "linux/videodev2.h"

#define sizenum 400
#define pai 3.1415926

//double timeLoad;//
int model=1;
bool changemode=1;
std::vector<basketball_msgs::ballElem> balls;
std::vector<basketball_msgs::cylinderElem> cylinders;

vector<Rect> decv;
vector<Rect> decv_blue;
detector mydec("cylinder");
detector mydec_blue("blue");

int rpdiar()  
{
	int argc;
	char **argv;
	return driver_base::main<HokuyoNode>(argc, argv, "hokuyo_node");
}

inline float pos2angle(int pos)
{
	//int sign=pos>256?1:-1;
	return 180*(pos)/512.0;
} 

inline float lrmin_dist(std::vector<float> vec,int mid,int dx,int& realaddr)
{
	float min_dist=999;
	for(int i=0;i<dx;i++)
	{
		if(vec[mid+i]<min_dist)
		{
			min_dist=vec[mid+i];
			realaddr=mid+i;
		}
		if(vec[mid-i]<min_dist)
		{
			min_dist=vec[mid-i];
			realaddr=mid-i;
		}
	}
	return min_dist;
}

bool serviceCallBack(basketball_msgs::visionDate::Request &req,
	basketball_msgs::visionDate::Response &rep)
{

	std::cout << "modle=" << req.model << std::endl;
	changemode = (model != req.model);
	model = req.model;
	
	//rep.timeLoad = timeLoad;
	std::cout << "[ serviceCallBack success ]" << std::endl;
	return true;
}

inline void rpdiarfilter_maxd(std::vector<float> vec,float thresh,
						std::vector<int>& mask,float maxdist,std::vector<int>& items,
						std::vector<int>& dlr,std::vector<int>& lefts,
						std::vector<int>& rights);
/*雷达使用方法
ls -l /dev/ttyACM0
sudo chmod a+rw /dev/ttyACM0
新建终端
roscore 
新建终端
rosparam set hokuyo_node/calibrate_time false
rosparam set hokuyo_node/port /dev/ttyACM0
*/



int main(int argc, char **argv)
{
	thread th1(rpdiar);
	th1.detach();
	ros::init(argc, argv, "vision");	
	ros::NodeHandle node;
	ros::ServiceServer service=node.advertiseService("visionDate", serviceCallBack);//回调函数，用于调整model值
	ros::Publisher ball_pub = node.advertise<basketball_msgs::balls>("/vision/balls", 100);
	ros::Publisher cylinder_pub = node.advertise<basketball_msgs::cylinders>("/vision/cylinders", 100);
	//采用Publisher发布球体和标定柱信息

	string cfgfile = "/home/hisfog/yolov3/darknet-master/cfg/yolov3-voc.cfg";
	string weightfile = "/home/hisfog/yolov3/darknet-master/backup/yolov3-voc_final.weights";
	network *net=load_network((char*)cfgfile.c_str(),(char*)weightfile.c_str(),0);

	vector<string> classNamesVec;
	ifstream classNamesFile("/home/hisfog/yolov3/darknet-master/data/coco.names");

	if (classNamesFile.is_open()){
		string className = "";
		while (getline(classNamesFile, className))
			classNamesVec.push_back(className);
	}

	wmj::UsbCaptureWithThread vdo("/dev/video1",1280,720);
	
	vdo.setBrightness(120);
	vdo.setGamma(30);
	Mat img;
	int getball=0;
	std::vector<int> items;
	std::vector<int> dlr;
	std::vector<float> items_dist;
	std::vector<int> lefts;
	std::vector<int> rights;

	ros::Rate rate(30);
	
	
	while(1)
	{
		balls.clear();
		cylinders.clear();
		vdo>>img;
		resize(img, img, Size(sizenum, sizenum * img.rows / img.cols), 0, 0, INTER_LINEAR);
		if(!img.empty() && model==1)//球体识别
		{
		
			vector<Rect> rects;
			vector<int> classes;
			getPredictions(img,rects,net,classNamesVec,classes);
			basketball_msgs::ballElem mainball;
			if(dists.size())
			{
				mainball.type=-1;
				mainball.distance=dists[256];
				balls.push_back(mainball);
			}
			
			std::vector<int> mask(512);
			if(dists.size())
			{
				items.clear();
				dlr.clear();
				items_dist.clear();
				lefts.clear();
				rights.clear();
				
				rpdiarfilter_maxd(dists,0.1,mask,3,items,dlr,lefts,rights);
				
				for(int i=0;i<items.size()-1;i++)
				{
						items_dist.push_back( dists[items[i]] );
				}
				for(int i=0;i<items.size();i++)
				{
					if(items_dist[i]>=0.2)
					cout<<"####item "<<i<<" theta "<<items[i]<<" dist "<<items_dist[i]<<" dlr "<<dlr[i]<<endl;				
				}

				
			}

			for(int i=0;i<rects.size();i++)
			{
				basketball_msgs::ballElem realball;
				if(classes[i] == 0 || classes[i] == 1)
					realball.type = 0;//篮球
				else if(classes[i] = 2 || classes[i] == 3)
					realball.type = 1;//排球

				
				realball.distance=-1;//默认距离为-1
				int centerx=rects[i].x + cvRound(rects[i].width / 2.0);
				int centery=rects[i].y + cvRound(rects[i].height / 2.0);
				int dx=centerx-sizenum/2;
				int addr=512*atan((dx)*0.6494/200)/pai;
				//int addr=512*atan((dx)*tan(angle*pai/180.0f)/200)/pai
				//angle是视角的一半，200是sizenum/2
				//该公式使用前提是相机水平放置，且相机中心与雷达中心在一条竖直线上
				//实际使用中存在误差，需要调整addr或者装配位置
				
				int img2ldaddr=256-addr;	
				int min_addr=-1;
				float thetax=pos2angle(256-addr);//左边>90,右边小于90
				float distancex=dists[img2ldaddr]<4 ? dists[img2ldaddr] : lrmin_dist(dists,img2ldaddr,5,min_addr);//dists[img2ldaddr];//real_dist>=2.8?10:real_dist;
				float thetax_move=atan( distancex*sin( (thetax-90)*pai/180 )/( 0.2+distancex*cos( (thetax-90)*pai/180) ) );//0.2是雷达到底盘中心的距离
				thetax_move=thetax_move*180/pai;
				realball.theta=thetax_move;
				realball.distance=distancex;
				cout<<"ball "<<realball.type<<" distance="
				    <<distancex<<" theta="<<thetax_move<<" ldaddr="<<img2ldaddr
				    <<" dx="<<dx<<" min_addr="<<min_addr<<endl;

				balls.push_back(realball);
			}
			
			basketball_msgs::balls balls_data;
			balls_data.balls = balls;
			ball_pub.publish(balls_data);

			


			imshow("result",img);

			if(waitKey(3)==27)break;
			//ros::spinOnce();
		}


		if(!img.empty() && model==2)//标定住
		{
			cylinders.clear();
			mydec.setimg(img);
			mydec_blue.setimg(img);
			mydec.setmode(25,3,2,12, 90,165,80,255,0,255,   90,165,80,255,0,255,0,0,0,0,0,0);
			mydec_blue.setmode(20,3,5,12, 180,265,90,255,0,255,   180,260,90,255,0,255,0,0,0,0,0,0);
			//mydec_blue.setmode(20,3,5,12, 190,230,90,255,0,255,   190,230,90,255,0,255,0,0,0,0,0,0);
			//mydec.setmode(30,2,4,12, 118,180,70,255,0,255,   118,180,70,255,0,255,0,0,0,0,0,0);//erode_size=4
			mydec.mainfunc(decv);
			mydec_blue.mainfunc(decv_blue);


			std::vector<int> mask(512);
			if(dists.size())
			{
				items.clear();
				dlr.clear();
				items_dist.clear();
				lefts.clear();
				rights.clear();
				rpdiarfilter_maxd(dists,0.1,mask,3,items,dlr,lefts,rights);
				
				for(int i=0;i<items.size()-1;i++)
				{
					
					items_dist.push_back( dists[items[i]] );
							
				}
					
				for(int i=0;i<items.size();i++)
				{
					if(items_dist[i]>0.2)
					cout<<"####item "<<i<<" theta "<<items[i]<<" dist "<<items_dist[i]<<" dlr "<<dlr[i]<<endl;				
				}
				cout<<"mid_dist="<<dists[256]<<endl;
			}

			unsigned int blue_k=0;
			for(unsigned int i = 0; i< decv.size(); i++ )
			{
				int centerx=decv[i].x + cvRound(decv[i].width / 2.0);
				int centery=decv[i].y + cvRound(decv[i].height / 2.0);
				int dx=centerx-sizenum/2;
				if(fabs(dx)>=160)continue;
				
				int flag_blue_green=0;
				
				if(decv_blue.size())
				{
					for(unsigned int k=0;k<decv_blue.size();k++)
					{
						int centerx_blue=decv_blue[k].x + cvRound(decv_blue[k].width / 2.0);
						//cout<<"###fabs(centerx-centerx_blue)= "<<fabs(centerx-centerx_blue)<<endl;
						if(fabs(centerx-centerx_blue)<=10)
						{
							flag_blue_green=1;
							blue_k=k;
						}
					}
				}
				if(flag_blue_green==0)continue;
				//如果存在某个绿色色块和某个蓝色色块的中心在一条竖直线上，则判定为标定住
					
				basketball_msgs::cylinderElem realcylinder;
				realcylinder.distance=-1;

				int addr=512*atan((dx)*sqrt(3)/600)/pai;
				//int addr=512*atan((dx)*tan(angle*pai/180.0f)/200)/pai
				//angle是视角的一半，200是sizenum/2
				//该公式使用前提是相机水平放置，且相机中心与雷达中心在一条竖直线上
				//实际使用中存在误差，需要调整addr或者装配位置
					
				int img2ldaddr=256-addr;
					
						
				int min_addr=-1;
				float thetax=pos2angle(256-addr);//左边>90,右边小于90
				float distancex=dists[img2ldaddr]<4 ? dists[img2ldaddr] : lrmin_dist(dists,img2ldaddr,30,min_addr);
				float thetax_move=atan( distancex*sin( (thetax-90)*pai/180 )/( 0.2+distancex*cos( (thetax-90)*pai/180) ) );//0.2是雷达到底盘中心的距离
				thetax_move=thetax_move*180/pai;
				realcylinder.theta=thetax_move;
				realcylinder.distance=distancex;
				cout<<"cylinder "<<i<<" distance="
					<<distancex<<" theta="<<thetax_move<<" ldaddr="<<img2ldaddr<<" dx="<<dx<<" min_addr="<<min_addr<<endl;

					
				cylinders.push_back(realcylinder);

				rectangle(img,decv[i],Scalar(255,0,0), 1, 1);
				rectangle(img,decv_blue[blue_k],Scalar(255,0,0),1,1);
				putText(img,"item",decv[i].tl(),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),2,4);
			}
			
				basketball_msgs::cylinders cylinders_data;
				cylinders_data.cylinders = cylinders;
				cylinder_pub.publish(cylinders_data);
				imshow("result",img);
				if(waitKey(3)==27)break;
				//ros::spinOnce();
		}
	}

	

	free_network(net);
	waitKey(0);
	

	return 0;
}



//记录雷达扫到的所有物体的左右和中心
void rpdiarfilter_maxd(std::vector<float> vec,float thresh,std::vector<int>& mask,
						float maxdist,std::vector<int>& items,std::vector<int>& dlr,
						std::vector<int>& lefts,std::vector<int>& rights)//thresh可以设置为最大球的半径
{
	int isin=0;
	int l=0,r=0;
	for(int i=0;i<vec.size()-1;i++)
	{
		if(vec[i]<=maxdist)
		{	
			if(fabs(vec[i]-vec[i+1])<=thresh)
			{
				mask[i]=0;
				if(isin==0)
				{
					l=i;
					isin=1;
					
				}
			}
			else
			{
				mask[i]=1;
				if(isin==1)
				{
					r=i;
					items.push_back((l+r)/2);
					dlr.push_back(r-l);
					lefts.push_back(l);
					rights.push_back(r);
					isin=0;
				}
			}
		}
		
	}
	mask[vec.size()-1]=1;
	return ;
}







