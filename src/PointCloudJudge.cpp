#include "PointCloudJudge.h"

bool PointCloudJudge::judgeCenter(const pcl::PointCloud<PointT>::ConstPtr & cloud_cptr,
    								    pcl::PointCloud<PointT>::Ptr      & cloud_filtered,
                                        PointT                            & center)
{
	filter(cloud_cptr, cloud_filtered);

    int cloud_filtered_size = static_cast<int>( cloud_filtered->points.size() );
    if( cloud_filtered_size < 50 )
    {
        printf("%s\n", "point cloud size after filter too little");
        return false;       
    }

    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (10000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int max_locate = -1;
    int max_size   = -1;
    for (std::vector< pcl::PointIndices >::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        //找到聚类中，拥有最大数量的聚类
        if( max_size < static_cast<int>(it->indices.size()))
        {
            max_size   = static_cast<int>(it->indices.size());
            max_locate = static_cast<int>(it - cluster_indices.begin());
        }
    }
    /*
    printf("max point cluster size %d\n", max_size);
    */
    double max_filtered_rate = (max_size + 0.0)/ cloud_filtered_size ; 
  
    if( max_filtered_rate < 0.3)
    {
        //找到最大聚类的点的数量，占矩形内点数目比例太少
        printf("max_filtered_rate < 0.3, can not find cylinder\n");
        return false;
    }

    pcl::PointCloud<PointT>::Ptr   cloud_cy_ptr(new pcl::PointCloud<PointT>);
    for(int i = 0; i < max_size; i++)
    {
        cloud_cy_ptr->points.push_back(cloud_filtered->points[ cluster_indices[max_locate].indices[i] ]);
    }
    cloud_cy_ptr->width = cloud_cy_ptr->points.size ();
    cloud_cy_ptr->height = 1;
    cloud_cy_ptr->is_dense = true;
    /*
    cloud_filtered = cloud_cy_ptr;
    */
    computeCenter(cloud_cy_ptr, center);
    return true;
} 

void PointCloudJudge::computeCenter(const pcl::PointCloud<PointT>::ConstPtr & cloud_cptr,
                             			  PointT                            & real_center)
{

    Eigen::Vector4f centroid;

    // std::sort(cloud_cptr->points.begin(), cloud_cptr->points.end(),[](PointT a, PointT b){return (a.z < b.z);});
    // std::sort(cloud_cptr->points.begin(), cloud_cptr->points.end());

    // myCompute3DCentroid(*cloud_cptr, centroid);
    pcl::compute3DCentroid(*cloud_cptr, centroid);

    real_center.x = centroid[0];
    real_center.y = centroid[1];
    real_center.z = centroid[2];
}

void PointCloudJudge::filter(const pcl::PointCloud<PointT>::ConstPtr & cloud_cptr,
                      			   pcl::PointCloud<PointT>::Ptr      & cloud_ptr)
{
    //voxelgrid并不是产生球面空洞的原因
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (cloud_cptr);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_ptr);
}

// // template <typename PointT> inline
// void PointCloudJudge::myCompute3DCentroid (const pcl::PointCloud<PointT> &cloud, 
//                         Eigen::Vector4f &centroid)
// {
//  // Initialize to 0
//     centroid.setZero ();
//     if (cloud.points.empty ()) 
//        return;
//     // For each point in the cloud
//     int cp = 0;

//       // If the data is dense, we don't need to check for NaN
//     if (cloud.is_dense)
//     {
//         for (size_t i = 0; i < cloud.points.size(); ++i)
//         centroid += cloud.points[i].getVector4fMap ();
//         centroid[3] = 0;
//         centroid /= cloud.points.size ();
//     }
//      // NaN or Inf values could exist => check for them
//     else
//     {
//         for (size_t i = 0; i < cloud.points.size(); ++i)
//         {
//         // Check if the point is invalid
//         if (!pcl_isfinite (cloud.points[i].x) || 
//             !pcl_isfinite (cloud.points[i].y) || 
//              !pcl_isfinite (cloud.points[i].z))
//             continue;

//         centroid += cloud.points[i].getVector4fMap ();
//         cp++;
//         }
//         centroid[3] = 0;
//         centroid /= cp;
//     }
// }
