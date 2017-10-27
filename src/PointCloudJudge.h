#ifndef POINTCLOUDJUDGE_H
#define POINTCLOUDJUDGE_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common_headers.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/functional.hpp>
#include <boost/shared_ptr.hpp>

#include <vector>
#include <string>
#include <sstream>
#include <iostream>

#include <cmath>
#include <cstdio>


typedef pcl::PointXYZ PointT;

class PointCloudJudge
{

private:

    void computeCenter(const pcl::PointCloud<PointT>::ConstPtr & cloud_cptr,
                             PointT                            & real_center);

    void filter(const pcl::PointCloud<PointT>::ConstPtr & cloud_cptr,
                      pcl::PointCloud<PointT>::Ptr      & cloud_ptr);
    
public:

    bool judgeCenter(const pcl::PointCloud<PointT>::ConstPtr & cloud_cptr,
    								       pcl::PointCloud<PointT>::Ptr      & cloud_filtered,
                           PointT                            & center);
};

#endif // POINTCLOUDJUDGE_H