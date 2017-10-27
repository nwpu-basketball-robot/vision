#ifndef JUDGE_H
#define JUDGE_H

#include "faster_rcnn.h"
#include "PointCloudJudge.h"
#include "cylinderFindImage.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

//boost
#include <boost/functional.hpp>
#include <boost/shared_ptr.hpp>

#include <map>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <utility>

/// [headers]
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
/// [headers]

#include <cstdio>
#include <cmath>
#include <cstdlib>

#include <signal.h>

#include "ros/ros.h"
#include "basketball_msgs/ballElem.h"
#include "basketball_msgs/cylinderElem.h"
#include "basketball_msgs/visionDate.h"

typedef pcl::PointXYZ PointT;

class MyFileLogger: public libfreenect2::Logger
{
private:
  std::ofstream logfile_;
public:
  MyFileLogger(const char *filename)
  {
    if (filename)
      logfile_.open(filename);
    level_ = Debug;
  }
  bool good()
  {
    return logfile_.is_open() && logfile_.good();
  }
  virtual void log(Level level, const std::string &message)
  {
    logfile_ << "[" << libfreenect2::Logger::level2str(level) << "] " << message << std::endl;
  }
};


class Judge
{
    private:

        Detector           detector1;
        Detector           detector2;
        FindCylinder       findCylinder;
        PointCloudJudge    pointCloudJudge;

        ros::NodeHandle    node_;
        ros::ServiceServer service_ ;

        bool initOK;
        bool isContinue;
        bool changeModel;
        int  missCount;
        
        libfreenect2::Freenect2        freenect2;
        libfreenect2::Freenect2Device *dev;
        libfreenect2::PacketPipeline  *pipeline;

        int                                 model;
        double                              timeLoad;
        std::vector< basketball_msgs::ballElem >     balls;
        std::vector< basketball_msgs::cylinderElem > cylinders;

        int iLowH;
        int iHighH;
        int iLowS;
        int iHighS;
        int iLowV;
        int iHighV;
        int rectArea;

    private:

        bool   init();
        double computeTheta(const cv::Point & center, 
                            const double & fx,
                            const double & cx);
        
        void   getPointCloud(const libfreenect2::Frame       & undistorted,
                             const libfreenect2::Registration* registration,
                             const cv::Rect                  & rect,
                             const cv::Mat                   & regImageGray,
                             pcl::PointCloud<PointT>::Ptr    & cloud_ptr);

        bool serviceCallBack(basketball_msgs::visionDate::Request  &req,
                             basketball_msgs::visionDate::Response &rep);

    public:

        Judge(const ros::NodeHandle & n);
        ~Judge();

        bool run();
};

#endif