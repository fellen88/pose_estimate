//字符串
#include <string>
//opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//cv_bridge
#include <cv_bridge/cv_bridge.h>
//tf坐标系变换
#include <tf/transform_broadcaster.h>
//滤波
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
//特征
#include <pcl/features/normal_3d.h>
//image transport
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
//算法时间测试
#include <pcl/common/time.h>
//欧式聚类
#include <pcl/segmentation/extract_clusters.h>

#include "rviz_visualizer.h"
#include "3d_features.h"
#include "3d_registration.h"

#ifndef POSE_ESTIMATE_H
#define POSE_ESTIMATE_H  
class pose_estimate
{
    public:
    std::string label;

    rviz_visualizer rviz_v_;
    //pcl_visualizer pcl_v_;
    registration registration_;

    //ROS图像转OpenCV变量
    cv_bridge::CvImagePtr depth_ptr, mask_ptr;

    Eigen::Vector3d euler_Angle;
    Eigen::Matrix3d Rotation_matrix;

    //PairAlign transformation matrix
    Eigen::Matrix4f Pre_PairAlign_Transformation;
    Eigen::Matrix4f PairAlign_Transformation;
    Eigen::Matrix4f GlobalTransformation;
    Eigen::Matrix4f UpsideDownTransformation;
    Eigen::Matrix4f GlobalTransformationForGrasp;  

    bool bUpdatingImage;
    bool bSaveImage;
    bool bSaveCloud;
    //调试标志
    bool DEBUG_VISUALIZER;

    pcl::PointCloud<pcl::PointXYZ>::Ptr CloudMask;
    pcl::PointCloud<pcl::PointXYZ>::Ptr CloudMaskAfterSample;
    pcl::PointCloud<pcl::PointXYZ>::Ptr CloudModel;
    pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPreProcess;
    pcl::PointCloud<pcl::PointXYZ>::Ptr CloudTransformedTarget;
    pcl::PointCloud<pcl::PointXYZ>::Ptr CloudEuclideanClusterAfterSample;
    pcl::PointCloud<pcl::PointXYZ>::Ptr CloudModelAfterSample;

    int depth_cols;
    int depth_rows;
    int camera_factor;
    double camera_cx;
    double camera_cy;
    double camera_fx;
    double camera_fy;

    double thetax;
    double thetay;
    double thetaz;
    // Eigen::Quaterniond quaternion;

    pose_estimate(const ros::NodeHandle& nodehandle, bool DebugVisualizer);
    int segmentation();
    int EuclideanCluster(const PointCloud::Ptr cloud_Segmentation, const PointCloud::Ptr cloud_EuclideanCluster);
    int Alignment();
    void udateTF();
    void Timer_PoseVisualization(const ros::TimerEvent& event);
};
#endif