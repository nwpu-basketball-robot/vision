
#ifndef CAMVISION_H
#define CAMVISION_H

#include "cylinderFindImage.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <map>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <utility>

#include <cstdio>
#include <cmath>
#include <cstdlib>

#include "ros/ros.h"
#include "basketball_msgs/cylinderElem.h"
#include "basketball_msgs/camVisionDate.h"

class CamVision
{
    private:
        
        FindCylinder       findCylinder;

        ros::NodeHandle    node_;
        ros::ServiceServer service_ ;
        
        double                              timeLoad;
        std::vector< basketball_msgs::cylinderElem > cylinders;

        int iLowH;
        int iHighH;
        int iLowS;
        int iHighS;
        int iLowV;
        int iHighV;
        int rectArea;

        double fx;
        double cx;
    
    private:

        void init();

        double computeTheta(const cv::Point & center);

        bool serviceCallBack(basketball_msgs::camVisionDate::Request  &req,
                             basketball_msgs::camVisionDate::Response &rep);

    public:

        CamVision(const ros::NodeHandle & n);
        ~CamVision();

        bool run();

};

#endif