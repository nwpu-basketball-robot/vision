#ifndef JUDGE_H
#define JUDGE_H

#include "PointCloudJudge.h"
#include "cylinderFindImage.h"
#include "ssd_detect.h"
#include <algorithm>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/functional.hpp>
#include <boost/shared_ptr.hpp>

#include <map>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <utility>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <cstdio>
#include <cmath>
#include <cstdlib>

#include <signal.h>

#include <ros/ros.h>
#include "basketball_msgs/ballElem.h"
#include "basketball_msgs/cylinderElem.h"
#include "basketball_msgs/visionDate.h"

typedef pcl::PointXYZ PointT;

class MyFileLogger : public libfreenect2::Logger
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
  Detector *detector;
  FindCylinder findCylinder;
  PointCloudJudge pointCloudJudge;

  ros::NodeHandle node_;
  ros::ServiceServer service_;

  bool initOK;
  bool isContinue;
  bool changeModel;
  int missCount;

  //we can use Receiver to replace freenect2, but there is no difference.
  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev;
  libfreenect2::PacketPipeline *pipeline;

  int model;
  double timeLoad;
  std::vector<basketball_msgs::ballElem> balls;
  std::vector<basketball_msgs::cylinderElem> cylinders;

  int iLowH_Basketball_Column;
  int iHighH_Basketball_Column;
  int iLowS_Basketball_Column;
  int iHighS_Basketball_Column;
  int iLowV_Basketball_Column;
  int iHighV_Basketball_Column;
  int rectArea_Basketball_Column;

  // int iLowH_Yellow_Column;
  // int iHighH_Yellow_Column;
  // int iLowS_Yellow_Column;
  // int iHighS_Yellow_Column;
  // int iLowV_Yellow_Column;
  // int iHighV_Yellow_Column;
  // int rectArea_Yellow_Column;

private:
  bool init();
  double computeTheta(const cv::Point &center,
                      const double &fx,
                      const double &cx);

  void getPointCloud(const libfreenect2::Frame &undistorted,
                     const libfreenect2::Registration *registration,
                     const cv::Rect &rect,
                     const cv::Mat &regImageGray,
                     pcl::PointCloud<PointT>::Ptr &cloud_ptr);

  bool serviceCallBack(basketball_msgs::visionDate::Request &req,
                       basketball_msgs::visionDate::Response &rep);

public:
  Judge(const ros::NodeHandle &n);
  ~Judge();

  bool run();
};

#endif //judge.h