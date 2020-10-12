#include "rviz_visualizer.h"


rviz_visualizer::rviz_visualizer(const ros::NodeHandle& node_handle): node_handle_(node_handle)
{
    marker_pub = node_handle_.advertise<visualization_msgs::Marker>("visualization_object", 10);
}

void rviz_visualizer :: updateBoundingBox(const PointCloud::Ptr cloud_src, bool downsample)
{ 
  // %Tag(MARKER_INIT)%
  visualization_msgs::Marker points, line_strip;
  points.header.frame_id = line_strip.header.frame_id = boundingbox_linelist.header.frame_id = "/PhoXi3Dscanner_sensor";
  points.header.stamp = line_strip.header.stamp = boundingbox_linelist.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = boundingbox_linelist.ns = "points_and_lines";
  points.action = line_strip.action = boundingbox_linelist.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = boundingbox_linelist.pose.orientation.w = 1.0;
 // %EndTag(MARKER_INIT)%

  // %Tag(ID)%
  points.id = 0;
  line_strip.id = 1;
  boundingbox_linelist.id = 2;
  // %EndTag(ID)%

  // %Tag(TYPE)%
  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  boundingbox_linelist.type = visualization_msgs::Marker::LINE_LIST;
  // %EndTag(TYPE)%

  // %Tag(SCALE)%
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.01;
  points.scale.y = 0.01;

  // LINE_STRIP/boundingbox_linelist markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;
  boundingbox_linelist.scale.x = 0.005;
  // %EndTag(SCALE)%

  // %Tag(COLOR)%
  // Points are red
  points.color.r = 1.0f;
  points.color.a = 1.0;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  // Line list is green
  boundingbox_linelist.color.g = 1.0;
  boundingbox_linelist.color.a = 0.6;
  // %EndTag(COLOR)%
  
  //体素下采样
  PointCloud::Ptr src (new PointCloud); //创建点云指针
  pcl::VoxelGrid<PointT> grid; //VoxelGrid 把一个给定的点云，聚集在一个局部的3D网格上,并下采样和滤波点云数据
  if (downsample) //下采样
  {
    grid.setLeafSize (0.007, 0.007, 0.007); //设置体元网格的叶子大小
        //下采样 源点云
    grid.setInputCloud (cloud_src); //设置输入点云
    grid.filter (*src); //下采样和滤波，并存储在src中
  }
  else //不进行下采样
  {
    src = cloud_src; //直接复制
  }
   //******************************OBB包围盒计算*************************************//
	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(src);
	feature_extractor.compute();

	std::vector <float> moment_of_inertia;
	std::vector <float> eccentricity;
	pcl::PointXYZ min_point_AABB;
	pcl::PointXYZ max_point_AABB;
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center;

	feature_extractor.getMomentOfInertia(moment_of_inertia);
	feature_extractor.getEccentricity(eccentricity);
	feature_extractor.getAABB(min_point_AABB, max_point_AABB);
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	feature_extractor.getEigenValues(major_value, middle_value, minor_value);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	feature_extractor.getMassCenter(mass_center);

	Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
	Eigen::Quaternionf quat(rotational_matrix_OBB);

	pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
	pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
	pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
	pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));

	Eigen::Vector3f p1(min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
	Eigen::Vector3f p2(min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
	Eigen::Vector3f p3(max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
	Eigen::Vector3f p4(max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
	Eigen::Vector3f p5(min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
	Eigen::Vector3f p6(min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
	Eigen::Vector3f p7(max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
	Eigen::Vector3f p8(max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

	p1 = rotational_matrix_OBB * p1 + position;
	p2 = rotational_matrix_OBB * p2 + position;
	p3 = rotational_matrix_OBB * p3 + position;
	p4 = rotational_matrix_OBB * p4 + position;
	p5 = rotational_matrix_OBB * p5 + position;
	p6 = rotational_matrix_OBB * p6 + position;
	p7 = rotational_matrix_OBB * p7 + position;
	p8 = rotational_matrix_OBB * p8 + position;

  // Create the vertices for the points and lines
  geometry_msgs::Point p;
  boundingbox_linelist.points.clear();
  //1 -> 2\4\5
  // The line list needs two points for each line
  p.x = p1(0);
  p.y = p1(1);
  p.z = p1(2);
  boundingbox_linelist.points.push_back(p);
  p.x = p2(0);
  p.y = p2(1);
  p.z = p2(2);
  boundingbox_linelist.points.push_back(p);
  // The line list needs two points for each line
  p.x = p1(0);
  p.y = p1(1);
  p.z = p1(2);
  boundingbox_linelist.points.push_back(p);
  p.x = p4(0);
  p.y = p4(1);
  p.z = p4(2);
  boundingbox_linelist.points.push_back(p);
  // The line list needs two points for each line
  p.x = p1(0);
  p.y = p1(1);
  p.z = p1(2);
  boundingbox_linelist.points.push_back(p);
  p.x = p5(0);
  p.y = p5(1);
  p.z = p5(2);
  boundingbox_linelist.points.push_back(p);

  //3 -> 2\4\7
  // The line list needs two points for each line
  p.x = p3(0);
  p.y = p3(1);
  p.z = p3(2);
  boundingbox_linelist.points.push_back(p);
  p.x = p2(0);
  p.y = p2(1);
  p.z = p2(2);
  boundingbox_linelist.points.push_back(p);
  // The line list needs two points for each line
  p.x = p3(0);
  p.y = p3(1);
  p.z = p3(2);
  boundingbox_linelist.points.push_back(p);
  p.x = p4(0);
  p.y = p4(1);
  p.z = p4(2);
  boundingbox_linelist.points.push_back(p);
  // The line list needs two points for each line
  p.x = p3(0);
  p.y = p3(1);
  p.z = p3(2);
  boundingbox_linelist.points.push_back(p);
  p.x = p7(0);
  p.y = p7(1);
  p.z = p7(2);
  boundingbox_linelist.points.push_back(p);

  //6 -> 2\5\7
  // The line list needs two points for each line
  p.x = p6(0);
  p.y = p6(1);
  p.z = p6(2);
  boundingbox_linelist.points.push_back(p);
  p.x = p2(0);
  p.y = p2(1);
  p.z = p2(2);
  boundingbox_linelist.points.push_back(p);
  // The line list needs two points for each line
  p.x = p6(0);
  p.y = p6(1);
  p.z = p6(2);
  boundingbox_linelist.points.push_back(p);
  p.x = p5(0);
  p.y = p5(1);
  p.z = p5(2);
  boundingbox_linelist.points.push_back(p);
  // The line list needs two points for each line
  p.x = p6(0);
  p.y = p6(1);
  p.z = p6(2);
  boundingbox_linelist.points.push_back(p);
  p.x = p7(0);
  p.y = p7(1);
  p.z = p7(2);
  boundingbox_linelist.points.push_back(p);

  //8 -> 4\5\7
  // The line list needs two points for each line
  p.x = p8(0);
  p.y = p8(1);
  p.z = p8(2);
  boundingbox_linelist.points.push_back(p);
  p.x = p4(0);
  p.y = p4(1);
  p.z = p4(2);
  boundingbox_linelist.points.push_back(p);
  // The line list needs two points for each line
  p.x = p8(0);
  p.y = p8(1);
  p.z = p8(2);
  boundingbox_linelist.points.push_back(p);
  p.x = p5(0);
  p.y = p5(1);
  p.z = p5(2);
  boundingbox_linelist.points.push_back(p);
  // The line list needs two points for each line
  p.x = p8(0);
  p.y = p8(1);
  p.z = p8(2);
  boundingbox_linelist.points.push_back(p);
  p.x = p7(0);
  p.y = p7(1);
  p.z = p7(2);
  boundingbox_linelist.points.push_back(p);
  //marker_pub.publish(points);
  //marker_pub.publish(line_strip);
  marker_pub.publish(boundingbox_linelist);
}

// void rviz_visualizer::publish_static_tf(const ros::Time& t,
//                                           const float3& trans,
//                                           const quaternion& q,
//                                           const std::string& from,
//                                           const std::string& to)
// {
//     geometry_msgs::TransformStamped msg;
//     msg.header.stamp = t;
//     msg.header.frame_id = from;
//     msg.child_frame_id = to;
//     msg.transform.translation.x = trans.z;
//     msg.transform.translation.y = -trans.x;
//     msg.transform.translation.z = -trans.y;
//     msg.transform.rotation.x = q.x;
//     msg.transform.rotation.y = q.y;
//     msg.transform.rotation.z = q.z;
//     msg.transform.rotation.w = q.w;
//     _static_tf_broadcaster.sendTransform(msg);
// }