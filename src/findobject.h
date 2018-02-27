#ifndef FINDOBJECT_H
#define FINDOBJECT_H

#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <sstream>
#include <iostream>
#include <string>
#include <cmath>

#include <object_detect/volleyballdata.h>
#include <object_detect/cylinderdata.h>
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%
#include "geometry_msgs/Point.h"

#include <sstream>

#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <vector>
#include <algorithm>

using namespace std ;

struct MyPoint
{
    double x, y, z;
};

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer(ros::NodeHandle node);
    ~SimpleOpenNIViewer();
private:
    ros::NodeHandle nh_ ;
    ros::ServiceServer ball_service_, cylinder_service_;
    //pcl::visualization::CloudViewer viewer_ ;
    geometry_msgs::Point current_ball_pos_, current_cylinder_pos_;
    tf::TransformListener listener;
    tf:: StampedTransform result;
    tf::Quaternion q;
    double robot_x, robot_y, robot_yaw,x0,y0;
    unsigned int id_ ;
    int ball_count, ball_control, cylinder_count, cylinder_control;
    int ball_miss_num, cylinder_miss_num;
    //double radius[4];
    bool ball_is_find, cylinder_is_find;
    bool if_volleyball, ball_or_cylinder;
    vector<MyPoint> census_ball_pos_, census_cylinder_pos_;
    //const static int PI = 3.14 ;
private:
    bool match(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) ;
    bool  match2(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
                 const pcl::PointCloud<pcl::Normal>::ConstPtr &cloud_normals2);

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) ;

    bool serviceCallBack(object_detect::volleyballdata::Request &req,
                                        object_detect::volleyballdata::Response &rep) ;
    bool serviceCallBack2(object_detect::cylinderdata::Request &req,
                                         object_detect::cylinderdata::Response &rep) ;
    pcl::PointCloud<pcl::Normal>::Ptr EstimateNorms(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void mainRun() ;
protected:
};

#endif // FINDOBJECT_H
