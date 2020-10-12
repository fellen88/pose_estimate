
//#include "pcl_visualizer.h"
#include "3d_features.h"

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
    //pcl_visualizer pcl_v_;

    //prePairAlign transformation matrix
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 MomentOfInertia_Transformation;

    registration(bool DebugVisualizer);
	fpfhFeature::Ptr compute_fpfh_feature(const PointCloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree);
	void SAC_IA_PareAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr transformed_cloud, Eigen::Matrix4f &SAC_transform, bool downsample);
    void prePairAlign(const PointCloud::Ptr cloud_src,const PointCloud::Ptr cloud_tgt, PointCloud::Ptr transformed_cloud, Eigen::Matrix4f &pre_transform, bool downsample);
    void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample);

};
#endif