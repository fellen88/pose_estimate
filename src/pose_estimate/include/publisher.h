#include <ros/ros.h>
//字符串
#include <string>
//ros std消息
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
//PCL specific includes
#include <sensor_msgs/PointCloud2.h>

class Publisher
{
  public:
  ros::NodeHandle node_handle_;
  //ros发布对象
  ros::Publisher ourpointcloud_pub; 
  sensor_msgs::PointCloud2 output;
  
  //发送变换已更新消息
  ros::Publisher Transformation_pub; 
  std_msgs::String TransformationMessage;

  ros::Publisher PointCloudError_pub; 

  Publisher(const ros::NodeHandle& node_handle);
  void init();
};

