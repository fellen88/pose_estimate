
#include "rviz_visualizer.h"
#include "pose_estimate.h"
//ros std��Ϣ
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>

//ROS���ݸ�ʽ��PCL���ݸ�ʽת��
#include <pcl_conversions/pcl_conversions.h>
//boostָ�����
#include <boost/make_shared.hpp> 
//��/����
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
  ros::V_Subscriber subs_;    //std::vector<Subscriber>  ��������
  
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