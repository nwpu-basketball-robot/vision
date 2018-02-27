#include "findobject.h"
#include <tf/transform_listener.h>
#include <tf/LinearMath//Quaternion.h>
#include <ros/ros.h>
#include <cmath>
SimpleOpenNIViewer::SimpleOpenNIViewer(ros::NodeHandle node):
    nh_(node),//viewer_("PCL OpenNI Viewer"),
    ball_miss_num(0), ball_is_find(false), ball_count(0), ball_control(0),if_volleyball(false),
    cylinder_miss_num(0), cylinder_is_find(false), cylinder_count(0), cylinder_control(0),ball_or_cylinder(false),
    id_(1)
{
    ball_service_ = nh_.advertiseService("volleyball_data",&SimpleOpenNIViewer::serviceCallBack,this) ;
    cylinder_service_ = nh_.advertiseService("cylinder_data", &SimpleOpenNIViewer::serviceCallBack2, this) ;
    ros::Time now = ros::Time::now();
    listener.waitForTransform("odom", "base_link",now, ros::Duration(0.05));
    mainRun();
}

SimpleOpenNIViewer::~SimpleOpenNIViewer()
{
    nh_.shutdown();
}

pcl::PointCloud<pcl::Normal>::Ptr SimpleOpenNIViewer::EstimateNorms(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    //cloud_normals2.clear();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    // Estimate point normals
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud);
    ne.setKSearch (60);
    ne.compute (*cloud_normals2);
    return cloud_normals2;
}


bool SimpleOpenNIViewer::match(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    pcl::ModelCoefficients::Ptr coefficients_Sphere (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_Sphere (new pcl::PointIndices);
        // Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg_Sphere;
        // Optional
	seg_Sphere.setOptimizeCoefficients (true);
        // Mandatory
	seg_Sphere.setModelType (pcl::SACMODEL_SPHERE);
	seg_Sphere.setMethodType (pcl::SAC_RANSAC);
	seg_Sphere.setDistanceThreshold (0.004);//yuan lai wei 0.001

	seg_Sphere.setInputCloud (cloud);
	seg_Sphere.segment (*inliers_Sphere, *coefficients_Sphere);

	if (inliers_Sphere->indices.size() == 0)
	{
		return false;
	}
	
    //对球的半径有选择的过滤
    if (coefficients_Sphere->values[3] < 0.095 || coefficients_Sphere->values[3] > 0.13)
		return false;

	if (inliers_Sphere->indices.size() < 100)
		return false;

    //cout << "ball size" << inliers_Sphere->indices.size() << endl;
    //to eliminate the interference from the outside
    ros::Time goal_time;
    string erro_string;
    listener.getLatestCommonTime("odom", "base_link",goal_time,&erro_string);
    listener.lookupTransform("odom", "base_link", goal_time, result);
    robot_x = result.getOrigin().x();
    robot_y = result.getOrigin().y();
    q =  result.getRotation();
    double w = q.getW(), x = q.getX() , y = q.getY(), z = q.getZ();
    robot_yaw = atan2(2*(w*z+ x*y),1-2*(y*y + z*z));
    x0 = robot_x + 0.11 * cos(robot_yaw) + (coefficients_Sphere->values[0]) * sin(robot_yaw) + (coefficients_Sphere->values[2]) * cos(robot_yaw);
    y0 = robot_y + 0.11 * sin(robot_yaw) + (coefficients_Sphere->values[2]) * sin(robot_yaw) - (coefficients_Sphere->values[0]) * cos(robot_yaw);
    if(x0<= -0.5 || x0 >= 8 || y0 >= 1.5 || y0 <= -12.5)
       return false;
    ball_count++;
    MyPoint tmp;
	tmp.x = coefficients_Sphere->values[0];
	tmp.y = coefficients_Sphere->values[1];
	tmp.z = coefficients_Sphere->values[2];
	census_ball_pos_.push_back(tmp);
    if (ball_count == 1)
	{
        ball_is_find = true;
            ball_count = 0;
		if (coefficients_Sphere->values[3] > 0.095 && coefficients_Sphere->values[3] < 0.114)
			if_volleyball = true;
		else
			if_volleyball = false;
		//viewer_.showCloud(cloud);
		std::ios::sync_with_stdio(false);
		cout << ">>>>>>>>>>>ball_size<<<<<<<<<<<<<<<" << inliers_Sphere->indices.size() << endl;
        cout << coefficients_Sphere->values[3] << endl;
		current_ball_pos_.x = census_ball_pos_[0].x;
		current_ball_pos_.y = census_ball_pos_[0].y;
		current_ball_pos_.z = census_ball_pos_[0].z;

		//cout << "ball has been finded" << endl;
		ROS_INFO("x:%lf y:%lf z:%lf", current_ball_pos_.x, current_ball_pos_.y, current_ball_pos_.z);
		double theta = atan2(current_ball_pos_.x, current_ball_pos_.z);
		cout << "theta:" << theta << endl;
		census_ball_pos_.clear();
    }
	else
	{
        ball_is_find = false;
		if_volleyball = false;
		current_ball_pos_.x = 0;
		current_ball_pos_.y = 0;
		current_ball_pos_.z = 0;
    }
    return true;
}

bool SimpleOpenNIViewer::match2(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
                const pcl::PointCloud<pcl::Normal>::ConstPtr &cloud_normals2)
{
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

    //cout << "Create the segmentation object for cylinder segmentation and set all the parameters" << endl;
    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);//没有太多噪点时可以设置为false
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (7000);//原来这里为10000
    seg.setDistanceThreshold (0.04);
    seg.setRadiusLimits (0.08, 0.12);
    seg.setInputCloud (cloud);
    seg.setInputNormals (cloud_normals2);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);

    if (inliers_cylinder->indices.size () == 0)
    {
        //PCL_ERROR ("Could not estimate a cylinder model for the given dataset.");
        return false;
    }
    if (coefficients_cylinder->values[6] < 0.08 || coefficients_cylinder->values[6] > 0.12)
        return false;
    if (inliers_cylinder->indices.size() < 1800 || inliers_cylinder->indices.size() > 3000)
        return false;

    //to eliminate the interference from the outside
    ros::Time goal_time;
    string erro_string;
    listener.getLatestCommonTime("odom", "base_link",goal_time,&erro_string);
    listener.lookupTransform("odom", "base_link", goal_time, result);
    robot_x = result.getOrigin().x();
    robot_y = result.getOrigin().y();
    q =  result.getRotation();
    double w = q.getW(), x = q.getX() , y = q.getY(), z = q.getZ();
    robot_yaw = atan2(2*(w*z+ x*y),1-2*(y*y + z*z));
    x0 = robot_x + 0.11 * cos(robot_yaw) + (coefficients_cylinder->values[0]) * sin(robot_yaw) + (coefficients_cylinder->values[2]) * cos(robot_yaw);
    y0 = robot_y + 0.11 * sin(robot_yaw) + (coefficients_cylinder->values[2]) * sin(robot_yaw) - (coefficients_cylinder->values[0]) * cos(robot_yaw);
    if(x0<= -0.5 || x0 >= 8.5 || y0 >= 1.5 || y0 <= -12.5)
       return false;
    cylinder_count++;
    MyPoint tmp;
    tmp.x = coefficients_cylinder->values[0];
    tmp.z = coefficients_cylinder->values[2];
    census_cylinder_pos_.push_back(tmp);
    if (cylinder_count == 1)
    {
        cylinder_is_find = true;
        cylinder_count = 0;
        //viewer_.showCloud(cloud);
        std::ios::sync_with_stdio(false);
        cout << ">>>>>>>>>>>cylinder_size<<<<<<<<<<<<<<<" << inliers_cylinder->indices.size() << endl;
        current_cylinder_pos_.x = census_cylinder_pos_[0].x;
        current_cylinder_pos_.z = census_cylinder_pos_[0].z;

        //cout << "ball has been finded" << endl;
        ROS_INFO("x:%lf z:%lf", current_cylinder_pos_.x, current_cylinder_pos_.z);
        double theta = -atan2(current_cylinder_pos_.x, current_cylinder_pos_.z);
        cout << "theta:" << theta << endl;
        census_cylinder_pos_.clear();
    }
    else
    {
        cylinder_is_find = false;
        current_cylinder_pos_.x = 0;
        current_cylinder_pos_.z = 0;
    }
    return true;
}

void SimpleOpenNIViewer::cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    cout << "id:" << id_++ <<endl;
    if (id_  == 10000)
        id_ = 0;

    if (ball_or_cylinder == false)
    {
        if (ball_control == 0)
        {
            cout << "waiting for on" << endl;
            return;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0, 4);
        pass.filter(*cloud_filtered);
	
        // 删去球顶点以上的坐标　build the condition
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
                                                                                   pcl::FieldComparison<pcl::PointXYZ>("y",pcl::ComparisonOps::GT, -0.25)));
        // build the filter
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setCondition (range_cond);
        condrem.setInputCloud (cloud_filtered);
        condrem.setKeepOrganized(true);
        condrem.filter (*cloud_filtered);

        //voxelgrid并不是产生球面空洞的原因
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud (cloud_filtered);
        vg.setLeafSize (0.01f, 0.01f, 0.01f);
        vg.filter (*cloud_filtered);
        if (cloud_filtered->points.size() < 300)
        {
            cout << "wait" << endl;
            return;
        }

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.04); // 4cm
        ec.setMinClusterSize (200);
        ec.setMaxClusterSize (10000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        ec.extract (cluster_indices);

        // viewer_.showCloud(cloud_filtered);
        int j = 0;
        int cnt = 0;
        bool ball_flag = false;

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            j++;
            //cout << "cloud_cluster_" << j << endl;
            if ( match(cloud_cluster))
            {
                ball_flag = true;
                //cout << ++cnt << " sphere have been detected " <<endl;
            }
        }
        if (ball_flag == false)
        {
            ball_miss_num++;
        }
        else
        {
            ball_miss_num = 0;
        }
        if (ball_miss_num == 25)
        {
            ball_is_find = false;
            ball_miss_num = 0;
            census_ball_pos_.clear();
        }
        if (! ball_is_find)
        {
            cout << "no ball" << endl;
        }
    }
    else
    {
            if (cylinder_control == 0)
            {
                cout << "waiting for on" << endl;
                return;
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(0, 4);
            pass.filter(*cloud_filtered);

            // delete some points of cylinder　build the condition
            pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
             /*range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
            pcl::FieldComparison<pcl::PointXYZ>("y",pcl::ComparisonOps::LT, 0.0)));*/
            range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
                                                                                        pcl::FieldComparison<pcl::PointXYZ>("y",pcl::ComparisonOps::GT, -0.6)));
            pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
            condrem.setCondition (range_cond);
            condrem.setInputCloud (cloud_filtered);
            condrem.setKeepOrganized(true);
            condrem.filter (*cloud_filtered);

            //voxelgrid并不是产生球面空洞的原因
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud (cloud_filtered);
            vg.setLeafSize (0.01f, 0.01f, 0.01f);
            vg.filter (*cloud_filtered);
            if(cloud_filtered->points.size() < 300)
            {
                cout << "fuck" << endl;
                return;
            }

            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
            tree->setInputCloud (cloud_filtered);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance (0.02); // 2cm
            ec.setMinClusterSize (500);
            ec.setMaxClusterSize (10000);
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud_filtered);
            ec.extract (cluster_indices);

            //viewer_.showCloud(cloud_filtered);
            int j = 0;
            int cnt = 0;
            bool cylinder_flag = false;

            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                    cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
                cloud_cluster->width = cloud_cluster->points.size ();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;
                j++;
                cloud_normals = EstimateNorms(cloud_cluster);
                if ( match2(cloud_cluster, cloud_normals))
                {
                    cylinder_flag = true;
                    //cout << ++cnt << " cylinder have been detected " <<endl;
                }
            }

            if (cylinder_flag == false)
            {
                cylinder_miss_num++;
            }
            else
            {
                cylinder_miss_num = 0;
            }
            if (cylinder_miss_num == 15)
            {
                cylinder_is_find = false;
                cylinder_miss_num = 0;
                census_cylinder_pos_.clear();
            }
            if (! cylinder_is_find)
            {
                cout << "no cylinder" << endl;
            }

    }
}


bool SimpleOpenNIViewer::serviceCallBack(object_detect::volleyballdata::Request &req,
                                         object_detect::volleyballdata::Response &rep)
{
    rep.x = current_ball_pos_.x;
    rep.y = current_ball_pos_.y;
    rep.z = current_ball_pos_.z;
    rep.theta = atan2( rep.x, rep.z);
    rep.has_ball = ball_is_find ;
    rep.if_volleyball = if_volleyball;
    ball_control = 1;
    ball_or_cylinder = req.ball_or_cylinder;
    return true;
}

bool SimpleOpenNIViewer::serviceCallBack2(object_detect::cylinderdata::Request &req,
                                          object_detect::cylinderdata::Response &rep)
{
    rep.x = current_cylinder_pos_.x;
    rep.z = current_cylinder_pos_.z;
    rep.theta = -atan2( rep.x, rep.z);
    rep.has_cylinder =cylinder_is_find ;
    cylinder_control = 1;
    ball_or_cylinder = req.ball_or_cylinder;
    return true;
}

void SimpleOpenNIViewer::mainRun()
{
    pcl::Grabber* interface = new pcl::OpenNIGrabber();

    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
            boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

    interface->registerCallback (f);

    interface->start ();

    ros::Rate r(5) ;

    while(ros::ok())
    {
        r.sleep();
        ros::spinOnce();
    }

    /*while (!viewer_.wasStopped())
    {
            r.sleep() ;
            ros::spinOnce() ;
    }*/

     interface->stop ();
}
