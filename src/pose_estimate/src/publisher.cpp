#include "publisher.h"

Publisher::Publisher(const ros::NodeHandle& node_handle): node_handle_(node_handle)
{
}

void Publisher::init()
{
  TransformationMessage.data = "TFUpdated";
  Transformation_pub = node_handle_.advertise<std_msgs::String>("transformation_update", 1);
  ourpointcloud_pub = node_handle_.advertise<sensor_msgs::PointCloud2> ("our_pointcloud", 1);
  PointCloudError_pub = node_handle_.advertise<std_msgs::Int8> ("PointCloudError", 1);
}