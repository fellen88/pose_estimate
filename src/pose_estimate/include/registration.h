
#include <ros/ros.h>

#include "MyPointRepresentation.h"

#include "pcl_visualizer.h"

//Åä×¼
#include <pcl/registration/icp.h> 
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/features/moment_of_inertia_estimation.h>


#ifndef REGISTRATION_H
#define REGISTRATION_H  

class registration
{
    public:

    bool DEBUG_VISUALIZER;
    pcl_visualizer pcl_v_;

    //prePairAlign transformation matrix
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 MomentOfInertia_Transformation;

    registration(bool DebugVisualizer);
    float prePairAlign(const PointCloud::Ptr cloud_src,const PointCloud::Ptr cloud_tgt, PointCloud::Ptr transformed_cloud, bool downsample);
    void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample);

};
#endif