
#include <listener.h>

Listener:: Listener(const ros::NodeHandle& node_handle, bool DebugVisualizer)
  : node_handle_(node_handle),pose_est_(node_handle, DebugVisualizer),publisher_(node_handle)
{
     pose_est_.bUpdatingImage = false;
     nCount = 0;
     publisher_.init();
}

void Listener::init()  //Listener类的init()方法
{
  // std::vector<Subscriber>.push_back()在容器尾部加入一个Subscriber对象(节点句柄的subscribe方法返回的) ，
  // boost::bind方法将一个函数转化成另一个函数
  //subs_.push_back(node_handle_.subscribe<std_msgs::String>("chatter", 1000, boost::bind(&Listener::chatterCallback, this, _1, "User 1")));
  subs_.push_back(node_handle_.subscribe<sensor_msgs::Image>("/segment/segment_image", 1, boost::bind(&Listener::Mask_Callback, this, _1, node_handle_)));
  //订阅深度图
  //cloud_sub_ = node_handle_.subscribe("/camera/depth_registered/points", 1, &Listener::Cloud_Callback, this);
  depth_sub_ = node_handle_.subscribe<sensor_msgs::Image>("/camera/depth/raw_image", 1 , boost::bind(&Listener::Depth_Callback, this, _1));
  //订阅RC消息，采集图像
  RobotControl_sub_ = node_handle_.subscribe<std_msgs::String>("/Ready", 1,  boost::bind(&Listener::CaptureImage_Callback, this, _1));
  //订阅label
  Label_sub_ = node_handle_.subscribe<std_msgs::String>("/segment/segment_class", 1, boost::bind(&Listener::Label_Callback, this, _1)); 
  //订阅相机发布点云数据
  //cloud_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, boost::bind(&Listener::Cloud_Callback, this, _1));
  //cloud_sub_ = node_handle_.subscribe("/camera/depth_registered/points", 1, &Listener::Cloud_Callback, this);//astra canera
  //cloud_sub_ = node_handle_.subscribe("/phoxi_camera/pointcloud", 1, &Listener::Cloud_Callback, this);
  
}

void Listener::CaptureImage_Callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("RobotControl Callback");
  ROS_INFO("I heard: [%s]", msg->data.c_str());

  //if(strcmp(msg->data.c_str(), "ready") == 0)
  {
    if(false == pose_est_.bSaveImage)
    {
      pose_est_.bSaveImage = true;
    }
    if(false == pose_est_.bSaveCloud)
    {
      pose_est_.bSaveCloud = true;
      
    }
  }
}

void Listener::Label_Callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Label Callback");
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  pose_est_.label =  msg->data.c_str();
}
   
void Listener::Depth_Callback(const sensor_msgs::ImageConstPtr& msg)
{
  if(true == pose_est_.bSaveImage)
  {
     pose_est_.bSaveImage = false; 
     pose_est_.bUpdatingImage = true;
     ROS_INFO("Start saving depth image");
     try
    {
      pose_est_.depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    pose_est_.bUpdatingImage = false;
    ROS_INFO("Stop saving depth image");

    int camera_factor = 1000;
    float camera_cx = 639.53; 
    float camera_cy = 363.99;
    float camera_fx = 640.49;
    float camera_fy = 640.49;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for(int ImgWidth = 0; ImgWidth < 720; ImgWidth++)
    {
      for(int ImgHeight = 0; ImgHeight < 1280; ImgHeight++ )
      {
        //获取深度图中对应点的深度值
        uint d = pose_est_.depth_ptr->image.at<uint>(ImgWidth,ImgHeight);

        //有效范围内的点
        if((d > 0.5*camera_factor) && (d < 1.2*camera_factor))
        {
          //计算这个点的空间坐标
          pcl::PointXYZ PointWorld;
          PointWorld.z = double(d)/camera_factor;
          //ROS_INFO("D = %f", d);
          PointWorld.x = (ImgHeight - camera_cx)*PointWorld.z/camera_fx;
          PointWorld.y = (ImgWidth - camera_cy)*PointWorld.z/camera_fy;
          cloud.points.push_back(PointWorld);
        }
      }
    }

    //Convert the cloud to ROS message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud,output);

    output.header.stamp = ros::Time::now();
    //output.header.frame_id = "/camera_rgb_optical_frame";
    output.header.frame_id = "PhoXi3Dscanner_sensor";
    publisher_.ourpointcloud_pub.publish(output);
  // //可视化深度图
  // //if(true == DEBUG_VISUALIZER)
  // {
  //   //Draw an example circle on the video stream
  //   if (pose_est_.depth_ptr->image.rows > 60 && pose_est_.depth_ptr->image.cols > 60)
  //     cv::circle(pose_est_.depth_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
  //   //Update GUI Window
  //   cv::imshow("depth image", pose_est_.depth_ptr->image);
  //   cv::waitKey(3);
  // }
  }
}

void Listener::Mask_Callback(const sensor_msgs::ImageConstPtr& msg, ros::NodeHandle& node_handle)
{
  ROS_INFO("Mask Callback");
  nCount++;
  ROS_INFO("##Post Estimation No.%d##",  nCount);
  try
  {
    pose_est_.mask_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  ros::Rate r(100); //100Hz,即等待10ms 
  //判断深度图是否正在更新
  while(pose_est_.bUpdatingImage)
  {
    ROS_INFO("Waiting for saving depth image");
    ros::spinOnce();
    r.sleep();
  }
  //关闭相机深度图订阅，1、锁定准备操作的深度图；2、减少配准过程订阅图像的资源利用率
  //depth_sub.shutdown();
  this->depth_sub_.shutdown();
  //深度图和mask都已准备就绪，进行匹配
  ROS_INFO("Start Alignment");
  std_msgs::Int8 ErrorCode;
  ErrorCode.data = pose_est_.Alignment();
  if(1 == ErrorCode.data || 2 == ErrorCode.data)
  {
     publisher_.PointCloudError_pub.publish(ErrorCode);
  }
  else
  {
    publisher_.Transformation_pub.publish(publisher_.TransformationMessage);

    //发布变换厚的物体模型
    pcl::PointCloud<pcl::PointXYZRGB> Color_model_transformed;
    pcl::copyPointCloud(*pose_est_.CloudTransformedTarget, Color_model_transformed);
    for (int j = 0; j < Color_model_transformed.size(); j++) 
    {
      Color_model_transformed.points[j].r = 0;
      Color_model_transformed.points[j].g = 255;
      Color_model_transformed.points[j].b = 0;
    }
    sensor_msgs::PointCloud2 model_transformed;
    pcl::toROSMsg(Color_model_transformed, model_transformed);
    model_transformed.header.stamp = ros::Time::now();
    //output.header.frame_id = "/camera_rgb_optical_frame";
    model_transformed.header.frame_id = "PhoXi3Dscanner_sensor";
    publisher_.ObjectModel_pub.publish(model_transformed);
  }
  
  //恢复订阅相机深度图
  //depth_sub_ = node_handle.subscribe("/camera/depth_registered/sw_registered/image_rect_raw", 1 , &Listener::Depth_Callback, this);
  depth_sub_ = node_handle.subscribe("/camera/depth/raw_image", 1 , &Listener::Depth_Callback, this);
}

//订阅点云及可视化
// void Listener::Cloud_Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
// {
//   if(true == pose_est_.bSaveCloud)
//   {
//     ROS_INFO("Start saving pointcloud");
//     //pcl::PointCloud<pcl::PointXYZRGB> cloud; // With color
//     pcl::PointCloud<pcl::PointXYZ> cloud; // Without color
//     pcl::PointCloud<pcl::PointXYZ> CloudAfterSample; // Without color
//     pcl::fromROSMsg(*msg, cloud); // sensor_msgs::PointCloud2 ----> pcl::PointCloud<T>
//     // pose_est_. registration_.pcl_v_.p->removePointCloud ("cloud test");
//     // pose_est_. registration_.pcl_v_.p->addPointCloud<pcl::PointXYZRGB>(cloud.makeShared(), "cloud test");
//     // pose_est_. registration_.pcl_v_.p->spin ();
//     //pcl::io::savePCDFileASCII("orbbec_cloud.pcd", cloud);	
//     //ROS_INFO("pointcloud saved");
    
//     //分割点云采样
//     pcl::VoxelGrid<PointT> grid; 
// 		grid.setLeafSize (0.001, 0.001, 0.001); //设置体元网格的叶子大小
// 		//下采样 源点云 
// 		grid.setInputCloud (cloud.makeShared()); //设置输入点云
// 		grid.filter (CloudAfterSample); //下采样和滤波，并存储在src中

//     //Convert the cloud to ROS message
//     sensor_msgs::PointCloud2 output;
//     pcl::toROSMsg(CloudAfterSample,output);

//     output.header.stamp = ros::Time::now();
//     //output.header.frame_id = "/camera_rgb_optical_frame";
//     output.header.frame_id = "PhoXi3Dscanner_sensor";
//     publisher_.ourpointcloud_pub.publish(output);
//     ROS_INFO("Stop saving pointcloud");
//     pose_est_.bSaveCloud = false;
//   }
// }


  
