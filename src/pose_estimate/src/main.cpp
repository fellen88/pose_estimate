/**************************************************
Copyright(c) 2018-2018 fellen All rights reserved. 
Author: fellen
Date:2018-08-20 
Description:1、Compute starting translation and rotation based on MomentOfInertiaEstimation descriptor
            2、LM-ICP Alignment   
            3、TF and boundingbox visualization                                                                                                                                       
**************************************************/
#include <ros/ros.h>
#include "listener.h"

void Timer_PoseVisualization(const ros::TimerEvent& event, Listener *listener)
{
  tf::Transform transform;
  static tf::TransformBroadcaster br;
  transform.setOrigin (tf::Vector3(listener->pose_est_.GlobalTransformation(0, 3), 
                                   listener->pose_est_.GlobalTransformation(1, 3), 
                                   listener->pose_est_.GlobalTransformation(2, 3)));
  transform.setRotation (tf::createQuaternionFromRPY (listener->pose_est_.thetax,
                                                      listener->pose_est_.thetay, 
                                                      listener->pose_est_.thetaz));
  //br.sendTransform (tf::StampedTransform(transform, ros::Time::now (),"/camera_rgb_optical_frame", "/object"));
  br.sendTransform (tf::StampedTransform(transform, ros::Time::now (),"/PhoXi3Dscanner_sensor", "/object"));
}

int main (int argc, char** argv)
{
  bool DEBUG_VISUALIZER = false;
  char* DEBUG_LABEL = NULL;
  if(2 == argc)
    DEBUG_LABEL = argv[1];
 
  if(DEBUG_LABEL == NULL)
  {
    DEBUG_VISUALIZER = false; 
    cout << "pcl_visualizer off" << endl;
    cout << "waiting for message..." << endl;
  }
  // 输入参数判断，“-v”可视化算法过程
  else if(strcmp(DEBUG_LABEL, "-v") == 0)
  {
    DEBUG_VISUALIZER = true; 
    cout << "pcl_visualizer on" << endl;
    cout << "waiting for message..." << endl;
  }
  else
  {
    cout << "rosrun parameters error!" << endl;
  }
  // Initialize ROS
  ros::init (argc, argv, "pose_estimate");
  ros::NodeHandle ros_nodehandle;
  // 创建Listener类
  Listener listener(ros_nodehandle, DEBUG_VISUALIZER);        
  listener.init();                          
  // Rviz物体姿态可视化timer
  ros::Timer timer = ros_nodehandle.createTimer(ros::Duration(0.1),boost::bind(&Timer_PoseVisualization, _1, &listener));
  ros::spin ();
}
