#include <ros/ros.h>
//�ַ���
#include <string>
//ros std��Ϣ
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
//PCL specific includes
#include <sensor_msgs/PointCloud2.h>

class Publisher
{
  public:
  ros::NodeHandle node_handle_;
  //ros��������
  ros::Publisher ourpointcloud_pub; 
  sensor_msgs::PointCloud2 output;
  
  //���ͱ任�Ѹ�����Ϣ
  ros::Publisher Transformation_pub; 
  std_msgs::String TransformationMessage;

  ros::Publisher PointCloudError_pub; 

  Publisher(const ros::NodeHandle& node_handle);
  void init();
};

