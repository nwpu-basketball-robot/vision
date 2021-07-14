#include "basketball_msgs/ballElem.h"
#include "basketball_msgs/cylinderElem.h"
#include "basketball_msgs/visionDate.h"
#include "basketball_msgs/balls.h"
#include "basketball_msgs/cylinders.h"

#include "class_timer.hpp"
#include "class_detector.h"
#include <opencv2/highgui/highgui_c.h>

#include "hokuyo_node.h"

#include <thread>
#include <memory>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "linux/videodev2.h"
#include <fstream>
#include <iostream>

#include "usb_capture_with_thread.h"

#define image_width  1280
#define image_height 720

struct hsv_params{
	int h_min=40;
	int h_max=100;
	int s_min=80;
	int s_max=130;
	int v_min=205;
	int v_max=175;
};

double timeLoad;
int model=1;
bool changemode=1;
std::vector<basketball_msgs::ballElem> balls;
std::vector<basketball_msgs::cylinderElem> cylinders;




int rpdiar(); 
double calAngle(cv::Mat cam,cv::Mat dis,int x,int y);

void rpdiarfilter_maxd(std::vector<float> vec,float thresh,
						std::vector<int>& mask,float maxdist,std::vector<int>& items,
						std::vector<int>& dlr,std::vector<int>& lefts,
						std::vector<int>& rights);


bool serviceCallBack(basketball_msgs::visionDate::Request &req,
	basketball_msgs::visionDate::Response &rep)
{

    std::cout << "modle=" << req.model << std::endl;
    changemode = (model != req.model);
    model = req.model;
    
    rep.timeLoad = timeLoad;
    //rep.balls = balls;
    //rep.cylinders = cylinders;

    //balls.clear();
    //cylinders.clear();

    std::cout << "[ serviceCallBack success ]" << std::endl;
    return true;
}
double threeIndex_min_distance(int index){
	double min_distance = 9;
	for(int i=-2;i<3;i++){
		if(dists[index+i]<min_distance ){
			if(index+i>231 &&index+i<291){
				min_distance=dists[index+i];
			}else if(dists[index+1]>0.20){
				min_distance = dists[index+i];
			}
			
		}
	}
	return min_distance;
}

double getBallCenterDistance(int index){
    double min_distance= 9 ;
    //find min distance in three index range
	int bias=-4;
    for(int i=0;i<20;i++){
		if(i==0)
			min_distance = threeIndex_min_distance(index);
		else{
			min_distance = threeIndex_min_distance(index+bias);
			bias=cvRound(pow(-1.0,i+1)*(i+1)*4)+bias;
		}
		if(0.1 <min_distance < 8.9)
			break;
		
    }
	return min_distance;
}

cv::Point find_cylinder_center_hsv(cv::Mat& img,hsv_params& cylinder_hsv_blue_params,hsv_params& cylinder_hsv_green_params);

int main(int argc, char **argv)
{
	thread th1(rpdiar);
	th1.detach();
	ros::init(argc, argv, "vision");	
	ros::NodeHandle node;
  	ros::ServiceServer service=node.advertiseService("visionDate", serviceCallBack);
    ros::Publisher ball_pub = node.advertise<basketball_msgs::balls>("/vision/balls", 100);
    ros::Publisher cylinder_pub = node.advertise<basketball_msgs::cylinders>("/vision/cylinders", 100);


	//load and store class labels by index 
	vector<string> classNamesVec;
    std::ifstream classNamesFile("/home/basketballrobot/workspace/src/vision/configs/basketball_v4_5class.names");
    if (classNamesFile.is_open()){
        string className = "";
        while (getline(classNamesFile, className))
            classNamesVec.push_back(className);
    }

	// yolo model configuration
	Config config_v3;
	config_v3.net_type = YOLOV3;
	config_v3.file_model_cfg = "/home/basketballrobot/workspace/src/vision/configs/basketball.cfg";
	config_v3.file_model_weights = "/home/basketballrobot/workspace/src/vision/configs/basketball.weights";
	config_v3.calibration_image_list_file_txt = "/home/basketballrobot/workspace/src/vision/configs/calibration_images.txt";
	config_v3.inference_precison = FP16;


	Config config_v4;
	config_v4.net_type = YOLOV4;
	config_v4.file_model_cfg = "/home/basketballrobot/workspace/src/vision/configs/basketball_final.cfg";
	config_v4.file_model_weights = "/home/basketballrobot/workspace/src/vision/configs/basketball_final.weights";
	config_v4.calibration_image_list_file_txt = "/home/basketballrobot/workspace/src/vision/configs/calibration_images.txt";
	config_v4.inference_precison = FP16;
	config_v4.detect_thresh=0.75;

	// detector declare 
	std::unique_ptr<Detector> detector_ = std::make_unique<Detector>();
	detector_->init(config_v4);

	// detected result vector declare
	std::vector<Result> res;
	Timer timer;
	cv::Mat img;  

	//int getball=0;
	std::vector<int> items;
	std::vector<int> items_fill;//
	std::vector<int> dlr;
	std::vector<int> dlr_fill;
	std::vector<float> items_dist;
	std::vector<int> lefts;
	std::vector<int> rights;

	ros::Rate rate(30);
	
	//camera parameters load
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	cv::FileStorage fs("/home/basketballrobot/workspace/src/vision/configs/out_camera_data.yml", cv::FileStorage::READ);
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

	//program params load
	cv::FileStorage params("/home/basketballrobot/workspace/src/vision/configs/config.xml",cv::FileStorage::READ);
	int deep_find_cylinder = false;
	hsv_params cylinder_hsv_blue_params;
	hsv_params cylinder_hsv_green_params;
	int middle_index=261;
	double indexPerDegree=2.45;
    double funcRatio = 0.000;
    int degree_bias=0;
	params["deep_find_cylinder"]>>deep_find_cylinder;
	params["blue_h_min"]>>cylinder_hsv_blue_params.h_min;
	params["blue_h_max"]>>cylinder_hsv_blue_params.h_max;
	params["blue_s_min"]>>cylinder_hsv_blue_params.s_min;
	params["blue_s_max"]>>cylinder_hsv_blue_params.s_max;
	params["blue_v_min"]>>cylinder_hsv_blue_params.v_min;
	params["blue_v_max"]>>cylinder_hsv_blue_params.v_max;

	params["green_h_min"]>>cylinder_hsv_green_params.h_min;
	params["green_h_max"]>>cylinder_hsv_green_params.h_max;
	params["green_s_min"]>>cylinder_hsv_green_params.s_min;
	params["green_s_max"]>>cylinder_hsv_green_params.s_max;
	params["green_v_min"]>>cylinder_hsv_green_params.v_min;
	params["green_v_max"]>>cylinder_hsv_green_params.v_max;

	params["middle_index"]>>middle_index;
	params["indexPerDegree"]>>indexPerDegree;
	params["funcRatio"]>>funcRatio;
	params["degree_bias"]>>degree_bias;
	params["model"]>>model;

	params.release();

	//wmj::UsbCaptureWithThread vdo1("/dev/video0",1280,720);
	//vdo1.setExposureTime(200);
	//vdo1.closeDevice();

	
	//camera init
	cv::VideoCapture vdo(0);
	
	//camera parameters set

	vdo.set(3,image_width);   // width
	vdo.set(4,image_height);	   // height
	//vdo.set(cv::CAP_PROP_AUTO_EXPOSURE,0.25);
	//vdo.set(cv::CAP_PROP_EXPOSURE,-8);
	//vdo.set(10, 120  ); // brightness     min: 0   , max: 255 , increment:1  
	//vdo.set(11, 50   ); // contrast       min: 0   , max: 255 , increment:1     
	//vdo.set(12, 70   ); // saturation     min: 0   , max: 255 , increment:1
	//vdo.set(13, 13   ); // hue         
	//vdo.set(14, 50   ); // gain           min: 0   , max: 127 , increment:1
	//vdo.set(15, -1  ); // exposure       min: -7  , max: -1  , increment:1
	//vdo.set(17, 5000 ); // white_balance  min: 4000, max: 7000, increment:1
	//vdo.set(28, 0    ); // focus          min: 0   , max: 255 , increment:5
	
	
	
	while(1)
	{
        balls.clear();
		vdo>>img;
		if(img.empty()){
			std::cout<<"the image is empty!"<<std::endl;
			continue;
		}
		// model 1
		if( model==1)//球体识别
		{
			timer.reset();
			//use yolo to detect balls,and the detection result is in res 
			detector_->detect(img, res);
			
			// mainball declare
			basketball_msgs::ballElem mainball;

			// dists is the distance array of radar's scan result
			if(dists.size())
			{
				mainball.type=-1;
				mainball.distance=dists[256];
				balls.push_back(mainball);
			}
			

			std::vector<int> mask(512);

			// radar compute distance
			// if(dists.size())
			// {
			// 	items.clear();
			// 	dlr.clear();
			// 	dlr_fill.clear();
			// 	items_fill.clear();
			// 	items_dist.clear();
			// 	lefts.clear();
			// 	rights.clear();
				
			// 	rpdiarfilter_maxd(dists,0.1,mask,3,items,dlr,lefts,rights);
				
			// 	for(int i=0;i<items.size();i++) 
			// 	{
			// 			items_dist.push_back( dists[items[i]] );
			// 	}
			// 	for(int i=0;i<items.size();i++)
			// 	{
			// 		if(items_dist[i]>=0.2)
			// 		cout<<"####item "<<i<<" radar_index "<<items[i]<<" dist "<<items_dist[i]<<" index_range_num "<<dlr[i]<<endl;				
			// 	}

				
			// }
                        cout<<"#########"<<endl;
			for(const auto &r : res)
			{
				cv::rectangle(img, r.rect, cv::Scalar(255, 0, 0), 2);
				basketball_msgs::ballElem realball;
				if(r.id == 0 || r.id == 1) //0,1代表排球，2,3代表篮球
					realball.type = 0;
				else if(r.id == 2 || r.id == 3) 
					realball.type = 1;
				else if(r.id==4)
					continue;
				realball.distance=-1;//默认距离为-1
				
				int ball_center_x = r.rect.x + cvRound(r.rect.width / 2.0);
				int ball_center_y = r.rect.y + cvRound(r.rect.height / 2.0);

				
				
				if(dists.size())
				{
					double robot_move_angle=-180;
					double camera_angle=-180;
					double radar_distance=-1;
					
					camera_angle = calAngle(cameraMatrix,distCoeffs,ball_center_x,ball_center_y);
				        
					int radar_distance_index = middle_index + cvRound(indexPerDegree*camera_angle); // 512/240=2.13333
                                        
					radar_distance = getBallCenterDistance(radar_distance_index);
					
					robot_move_angle = atan( radar_distance*sin( camera_angle*CV_PI/180 )/( 0.21+radar_distance*cos( camera_angle*CV_PI/180) ) )*180/CV_PI;//0.21是雷达到底盘中心的距离
                                        
					robot_move_angle = robot_move_angle-robot_move_angle*robot_move_angle*funcRatio-degree_bias;
					realball.theta=robot_move_angle;
					realball.distance=radar_distance;

					cout<<"ball "<<realball.type
                                                <<" distance_index="<<radar_distance_index
                                                <<" camera_angle="<<camera_angle
						<<" distance="<<radar_distance
						<<" angle="<<robot_move_angle<<endl;

				}

				
				balls.push_back(realball);
			}
			basketball_msgs::balls balls_data;
			balls_data.balls = balls;
			ball_pub.publish(balls_data);
			/*
			cv::namedWindow("result",0);
	        cv::resizeWindow("result",640,360);
			cv::imshow("result",img);
                        int key = cv::waitKey(1);
			if(key==27)break;
			
			if(key==97)middle_index++;
			if(key==100)middle_index--;
			if(key==119)indexPerDegree=indexPerDegree+0.01;
			if(key==115)indexPerDegree=indexPerDegree-0.01;
                        if(key==113)funcRatio=funcRatio-0.00001;
			if(key==101) funcRatio=funcRatio+0.00001;
                        if(key==103) degree_bias=degree_bias-1;
			if(key==104) degree_bias=degree_bias+1;
                        std::cout<<"middle_index : "<<middle_index<<std::endl;
                        std::cout<<"indexPerDegree : "<<indexPerDegree<<std::endl;
                        std::cout<<"funcRatio : "<<funcRatio<<std::endl;
                        std::cout<<"degree_bias : "<<degree_bias<<std::endl;
			*/
				
   		}
		if( model==2){
			cv::Point cylinder_center;
			if(deep_find_cylinder){
				detector_->detect(img, res);
				for(const auto &r : res)
				{
					if(r.id==4 ){
						cv::rectangle(img, r.rect, cv::Scalar(255, 0, 0), 2);
						cylinder_center.x = r.rect.x+r.rect.width/2;
						cylinder_center.y = r.rect.y+r.rect.height/2;
					}
					
				}
			}else{
				cylinder_center = find_cylinder_center_hsv(img,cylinder_hsv_blue_params,cylinder_hsv_green_params);
			}
			cylinders.clear();
			basketball_msgs::cylinderElem realcylinder;
			realcylinder.distance=-1;
			if(dists.size()){
				
				
				std::cout<<"center point is: "<<cylinder_center.x<<"  , "<<cylinder_center.y<<std::endl;

				double robot_move_angle=-180;
				double camera_angle=-180;
				double radar_distance=-1;

				camera_angle = calAngle(cameraMatrix,distCoeffs,cylinder_center.x,cylinder_center.y);
				        
				int radar_distance_index = middle_index + cvRound(indexPerDegree*camera_angle); // 512/240=2.13333
									
				radar_distance = getBallCenterDistance(radar_distance_index);
				
				robot_move_angle = atan( radar_distance*sin( camera_angle*CV_PI/180 )/( 0.21+radar_distance*cos( camera_angle*CV_PI/180) ) )*180/CV_PI;//0.21是雷达到底盘中心的距离
									
				robot_move_angle = robot_move_angle-robot_move_angle*robot_move_angle*funcRatio-degree_bias;
				realcylinder.theta=robot_move_angle;
				realcylinder.distance=radar_distance;
				
				cout<<"cylinder "
                        <<" distance_index="<<radar_distance_index
                        <<" camera_angle="<<camera_angle
						<<" distance="<<radar_distance
						<<" angle="<<robot_move_angle<<endl;
			/*	
				cv::namedWindow("img",0);
				cv::resizeWindow("img",640,360);
				cv::imshow("img",img);
				int key=cv::waitKey(1);
				if(key==27)  break;
			  */     

				
			}
            cylinders.push_back(realcylinder);
			basketball_msgs::cylinders cylinders_data;
			cylinders_data.cylinders = cylinders;
			cylinder_pub.publish(cylinders_data);
		}

	}

	return 0;
}


/**********************************************************************/
/*                         robot move angle                           */ 
/*    Firstly,according to camera parameters,compute the camera angle.*/
/*	Then,use math method to get the robot move angle based on the     */
/*	distance between camera center and robot center                   */                          
/**********************************************************************/
double calAngle(cv::Mat cam,cv::Mat dis,int x,int y){

    double fx=cam.at<double>(0,0);
    double fy=cam.at<double>(1,1);
    double cx=cam.at<double>(0,2);
    double cy=cam.at<double>(1,2);
    double k1=dis.at<double>(0);
    double k2=dis.at<double>(1);
    double p1=dis.at<double>(2);
    double p2=dis.at<double>(3);
    cv::Point2f pnt;
    vector<cv::Point2f>in;
    vector<cv::Point2f>out;
    in.push_back(cv::Point2f(x,y));
    //对像素点去畸变

    cv::undistortPoints(in,out,cam,dis,cv::noArray(),cam);
    
    pnt=out.front();
    double rx=(pnt.x-cx)/fx;
    double ry=(pnt.y-cy)/fy;

    double tanx=(rx);
    double tany=(ry);

    return -1*atan(rx)/CV_PI*180;
    //cout<< "xscreen: "<<x<<" xNew:"<<pnt.x<<endl;
    //cout<< "yscreen: "<<y<<" yNew:"<<pnt.y<<endl;
    //cout<< "angx: "<<atan((x-cx)/fx)/CV_PI*180<<" angleNew:"<<atan(rx)/CV_PI*180<<endl;
    //cout<< "angy: "<<atan((y-cy)/fy)/CV_PI*180<<" angleNew:"<<atan(ry)/CV_PI*180<<endl;
}


/**********************************************************************/
/*              find_cylinder_center using hsv method                 */ 
/*  args:   cv::Mat& img                                              */
/*	                                                                  */
/*	return: the cylinder center which is always in green part         */                          
/**********************************************************************/
cv::Point find_cylinder_center_hsv(cv::Mat& img,hsv_params& cylinder_hsv_blue_params,hsv_params& cylinder_hsv_green_params){
	cv::Mat hsv_img,binary_mask;
	cv::GaussianBlur(img,img,cv::Size(3,3),0,0);
	
	//convert to hsv space and get binary img in hsv range which you set in hsv_params 
	cv::cvtColor(img,hsv_img,CV_BGR2HSV);
	
	cylinder_hsv_blue_params.h_min=50;
	cylinder_hsv_blue_params.s_min=60;
	cylinder_hsv_blue_params.v_min=25;
	cylinder_hsv_blue_params.h_max=130;
	cylinder_hsv_blue_params.s_max=215;
	cylinder_hsv_blue_params.v_max=245;
	//cv::Scalar(cylinder_hsv_blue_params.h_min,cylinder_hsv_blue_params.s_min,cylinder_hsv_blue_params.v_min);
	//cv::Scalar(cylinder_hsv_blue_params.h_max,cylinder_hsv_blue_params.s_max,cylinder_hsv_blue_params.v_max);

	cv::inRange(hsv_img,
				cv::Scalar(cylinder_hsv_green_params.h_min,cylinder_hsv_green_params.s_min,cylinder_hsv_green_params.v_min),
				cv::Scalar(cylinder_hsv_blue_params.h_max,cylinder_hsv_green_params.s_max,cylinder_hsv_blue_params.v_max),
				binary_mask);

	//cv::imshow("blue_mask",binary_mask);
	
    // dilate and erode operation
	cv::Mat structureElement = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(21,21),cv::Point(-1,-1));
	cv::Mat structureElement1 = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5),cv::Point(-1,-1));
	cv::dilate(binary_mask,binary_mask,structureElement1);
	cv::erode(binary_mask,binary_mask,structureElement);
	cv::namedWindow("blue_mask_erode",0);
	cv::resizeWindow("blue_mask_erode",640,360);
	cv::imshow("blue_mask_erode",binary_mask);
	
	//find contours so as to get min area rects
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(binary_mask,contours,hierarchy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE,cv::Point());
	//draw contours
	cv::Point center_point;
	center_point.y=1023;
	for(int i=0;i<contours.size();i++){
		
		cv::RotatedRect rect = cv::minAreaRect(contours[i]);
		cv::Point2f corner_points[4];
		cv::Point current_point,top_point,bottom_point;
		rect.points(corner_points);
		current_point.x=int(corner_points[0].x+corner_points[2].x)/2;
		current_point.y=int(corner_points[0].y+corner_points[2].y)/2;
		top_point.x = int(corner_points[1].x+corner_points[2].x)/2 ;
		top_point.y = int(corner_points[0].y-corner_points[2].y)/5+corner_points[2].y;
		bottom_point.x = int(corner_points[0].x+corner_points[3].x)/2;
		bottom_point.y = int(corner_points[0].y-corner_points[2].y)*4/5+corner_points[2].y;

		//cylinder match conditons: bottom and top points is blue and middle(current) point is green
		if( (
			(hsv_img.at<cv::Vec3b>(top_point.y,top_point.x)[0]>cylinder_hsv_blue_params.h_min && hsv_img.at<cv::Vec3b>(top_point.y,top_point.x)[0]<cylinder_hsv_blue_params.h_max &&
		    hsv_img.at<cv::Vec3b>(top_point.y,top_point.x)[1]>cylinder_hsv_blue_params.s_min && hsv_img.at<cv::Vec3b>(top_point.y,top_point.x)[1]<cylinder_hsv_blue_params.s_max &&
			hsv_img.at<cv::Vec3b>(top_point.y,top_point.x)[2]>cylinder_hsv_blue_params.v_min && hsv_img.at<cv::Vec3b>(top_point.y,top_point.x)[2]<cylinder_hsv_blue_params.v_max) 
			||
			(hsv_img.at<cv::Vec3b>(bottom_point.y,bottom_point.x)[0]>cylinder_hsv_blue_params.h_min && hsv_img.at<cv::Vec3b>(bottom_point.y,bottom_point.x)[0]<cylinder_hsv_blue_params.h_max &&
		    hsv_img.at<cv::Vec3b>(bottom_point.y,bottom_point.x)[1]>cylinder_hsv_blue_params.s_min && hsv_img.at<cv::Vec3b>(bottom_point.y,bottom_point.x)[1]<cylinder_hsv_blue_params.s_max &&
			hsv_img.at<cv::Vec3b>(bottom_point.y,bottom_point.x)[2]>cylinder_hsv_blue_params.v_min && hsv_img.at<cv::Vec3b>(bottom_point.y,bottom_point.x)[2]<cylinder_hsv_blue_params.v_max)
			||
			(hsv_img.at<cv::Vec3b>(bottom_point.y-10,bottom_point.x)[0]>cylinder_hsv_blue_params.h_min && hsv_img.at<cv::Vec3b>(bottom_point.y-10,bottom_point.x)[0]<cylinder_hsv_blue_params.h_max &&
		    hsv_img.at<cv::Vec3b>(bottom_point.y-10,bottom_point.x)[1]>cylinder_hsv_blue_params.s_min && hsv_img.at<cv::Vec3b>(bottom_point.y-10,bottom_point.x)[1]<cylinder_hsv_blue_params.s_max &&
			hsv_img.at<cv::Vec3b>(bottom_point.y-10,bottom_point.x)[2]>cylinder_hsv_blue_params.v_min && hsv_img.at<cv::Vec3b>(bottom_point.y-10,bottom_point.x)[2]<cylinder_hsv_blue_params.v_max)
			
			
			)
			&&
			(
			(hsv_img.at<cv::Vec3b>(current_point.y,current_point.x)[0]>cylinder_hsv_green_params.h_min && hsv_img.at<cv::Vec3b>(current_point.y,current_point.x)[0]<cylinder_hsv_green_params.h_max &&
		    hsv_img.at<cv::Vec3b>(current_point.y,current_point.x)[1]>cylinder_hsv_green_params.s_min && hsv_img.at<cv::Vec3b>(current_point.y,current_point.x)[1]<cylinder_hsv_green_params.s_max &&
			hsv_img.at<cv::Vec3b>(current_point.y,current_point.x)[2]>cylinder_hsv_green_params.v_min && hsv_img.at<cv::Vec3b>(current_point.y,current_point.x)[2]<cylinder_hsv_green_params.v_max) 
			||
			(hsv_img.at<cv::Vec3b>(current_point.y+5,current_point.x+5)[0]>cylinder_hsv_green_params.h_min && hsv_img.at<cv::Vec3b>(current_point.y+5,current_point.x+5)[0]<cylinder_hsv_green_params.h_max &&
		    hsv_img.at<cv::Vec3b>(current_point.y+5,current_point.x+5)[1]>cylinder_hsv_green_params.s_min && hsv_img.at<cv::Vec3b>(current_point.y+5,current_point.x+5)[1]<cylinder_hsv_green_params.s_max &&
			hsv_img.at<cv::Vec3b>(current_point.y+5,current_point.x+5)[2]>cylinder_hsv_green_params.v_min && hsv_img.at<cv::Vec3b>(current_point.y+5,current_point.x+5)[2]<cylinder_hsv_green_params.v_max)  
			||
			(hsv_img.at<cv::Vec3b>(current_point.y-5,current_point.x-5)[0]>cylinder_hsv_green_params.h_min && hsv_img.at<cv::Vec3b>(current_point.y-5,current_point.x-5)[0]<cylinder_hsv_green_params.h_max &&
		    hsv_img.at<cv::Vec3b>(current_point.y-5,current_point.x-5)[1]>cylinder_hsv_green_params.s_min && hsv_img.at<cv::Vec3b>(current_point.y-5,current_point.x-5)[1]<cylinder_hsv_green_params.s_max &&
			hsv_img.at<cv::Vec3b>(current_point.y-5,current_point.x-5)[2]>cylinder_hsv_green_params.v_min && hsv_img.at<cv::Vec3b>(current_point.y-5,current_point.x-5)[2]<cylinder_hsv_green_params.v_max)  
			
			)

			

			

		){
			center_point.x=current_point.x;
			center_point.y=current_point.y;
			cv::drawContours(img,contours,i,cv::Scalar(0,0,255),2,8,hierarchy);
		}
	}
	
	return center_point;

}

cv::Point find_cylinder_center_deep(cv::Mat& img){

}

void rpdiarfilter_maxd(std::vector<float> vec,float thresh,std::vector<int>& mask,
						float maxdist,std::vector<int>& items,std::vector<int>& dlr,
						std::vector<int>& lefts,std::vector<int>& rights)//thresh可以设置为最大球的半径
{
    int isin=0;
    int l=0,r=0;
    for(int i=0;i<vec.size();i++)
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
    				//cout<<"rpdiarfilter left "<<l<<endl;
    				
    			}
    			//cout<<"delta="<<fabs(vec[i]-vec[i+1])<<endl;
    				//cout<<"nums "<<vec[i]<<endl;
    		}
    		else
    		{
    			//	cout<<"@@@@@delta="<<fabs(vec[i]-vec[i+1])<<endl;
    			mask[i]=1;
    			//cout<<"nums "<<vec[i]<<endl;
    			if(isin==1)
    			{
    				r=i;
    				//cout<<"rpdiarfilter lr "<<l<<" "<<r<<endl;
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
    //cout<<"nums "<<vec[vec.size()-1]<<endl;
    /*if(isin=1)
    {
    	r=vec.size()-1;
    	cout<<"rpdiarfilter right "<<l<<" "<<r<<endl;
    	isin=0;
    }
    */
    //cout<<"center="<<(r+l)/2<<" "<<"dist="<<vec[(r+l)/2]<<endl;
    return ;
}

int rpdiar()  
{
    int argc;
    char **argv;
    return driver_base::main<HokuyoNode>(argc, argv, "hokuyo_node");
}



/*雷达使用方法
ls -l /dev/ttyACM0
sudo chmod a+rw /dev/ttyACM0
新建终端
roscore 
新建终端
rosparam set hokuyo_node/calibrate_time false
rosparam set hokuyo_node/port /dev/ttyACM0
*/
	
	//把摄像头极限位置的偏角对应于dists中的索引测出，设置thresh保证不测到场外的物体距离，然后从左到右的顺序对应标签和角度距离
	//实际测得摄像头视角为60度，在雷达中对应于雷达中心的176个数据，即dists[171]~dists[341]
	//如果雷达测得球的中心在这个范围内就可以和图像对应
	//CV_PI/6=arctan(200/d),1/d=tan(W/6)/200,200是imgsize/2?
	//512*atan(fabs(dx)*sqrt(3)/(3×200))/3.1415926为实际角度的索引的解算公式，索引误差最大为10左右
	//实测，实际角度越大误差越大，应该是摄像头畸变导致，最大误差在3度左右，在0度时只存在装配误差，角度为中线偏角
	//需要一个/,, 13临近搜索矫正角度和距离

