
#include <ros/ros.h>

#include "MyPointRepresentation.h"

//tf坐标系变换
#include <tf/transform_broadcaster.h>
//Marker消息
#include <visualization_msgs/Marker.h>

#ifndef RVIZ_VISUALIZER_H
#define RVIZ_VISUALIZER_H  

class rviz_visualizer
{
    public:
    visualization_msgs::Marker boundingbox_linelist;
    ros::Publisher marker_pub;

    rviz_visualizer(const ros::NodeHandle& node_handle);
    void updateBoundingBox(const PointCloud::Ptr cloud_src, bool downsample);
    // void publish_static_tf(const ros::Time& t,
    //                         const float3& trans,
    //                         const quaternion& q,
    //                         const std::string& from,
    //                         const std::string& to);

    protected:
    ros::NodeHandle node_handle_;
    
};

#endif