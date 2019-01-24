
#include "rviz_visualizer.h"
#include "pose_estimate.h"
//ros std消息
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>

//ROS数据格式与PCL数据格式转换
#include <pcl_conversions/pcl_conversions.h>
//boost指针管理
#include <boost/make_shared.hpp> 
//点/点云
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
//image transport
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "publisher.h"


#ifndef LINSTENER_H
#define LINSTENER_H        

class Listener
{
  public:
  ros::NodeHandle node_handle_;
  ros::V_Subscriber subs_;    //std::vector<Subscriber>  向量容器
  
  ros::Subscriber MaskRCNN_sub_;
  ros::Subscriber RobotControl_sub_;
  ros::Subscriber RobotControl_sub1_;
  ros::Subscriber Label_sub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber update_sub_;
  ros::Subscriber depth_sub_;
  pose_estimate pose_est_;
  Publisher publisher_;
  int nCount;

  Listener(const ros::NodeHandle& node_handle, bool DebugVisualizer);

  void init();

  void Depth_Callback(const sensor_msgs::ImageConstPtr& msg);
  void Cloud_Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void Label_Callback(const std_msgs::String::ConstPtr& msg);
  void CaptureImage_Callback1(const std_msgs::Int8::ConstPtr& msg);
  void CaptureImage_Callback(const std_msgs::String::ConstPtr& msg);
  void Mask_Callback(const sensor_msgs::ImageConstPtr& msg, ros::NodeHandle& node_handle);
};
#endif