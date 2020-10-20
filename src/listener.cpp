
#include <listener.h>

Listener:: Listener(const ros::NodeHandle& node_handle, bool DebugVisualizer)
  : node_handle_(node_handle),pose_est_(node_handle, DebugVisualizer),publisher_(node_handle)
{
     pose_est_.bUpdatingImage = false;
     nCount = 0;
     publisher_.init();
}

void Listener::init()  //Listener���init()����
{
  // std::vector<Subscriber>.push_back()������β������һ��Subscriber����(�ڵ�����subscribe�������ص�) ��
  // boost::bind������һ������ת������һ������
  //subs_.push_back(node_handle_.subscribe<std_msgs::String>("chatter", 1000, boost::bind(&Listener::chatterCallback, this, _1, "User 1")));
  subs_.push_back(node_handle_.subscribe<sensor_msgs::Image>("/segment/segment_image", 1, boost::bind(&Listener::Mask_Callback, this, _1, node_handle_)));
  //�������ͼ
  //cloud_sub_ = node_handle_.subscribe("/camera/depth_registered/points", 1, &Listener::Cloud_Callback, this);
  depth_sub_ = node_handle_.subscribe<sensor_msgs::Image>("/camera/depth/raw_image", 1 , boost::bind(&Listener::Depth_Callback, this, _1));
  //����RC��Ϣ���ɼ�ͼ��
  RobotControl_sub_ = node_handle_.subscribe<std_msgs::String>("/Ready", 1,  boost::bind(&Listener::CaptureImage_Callback, this, _1));
  //����label
  Label_sub_ = node_handle_.subscribe<std_msgs::String>("/segment/segment_class", 1, boost::bind(&Listener::Label_Callback, this, _1)); 
  //�������������������
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
        //��ȡ���ͼ�ж�Ӧ������ֵ
        uint d = pose_est_.depth_ptr->image.at<uint>(ImgWidth,ImgHeight);

        //��Ч��Χ�ڵĵ�
        if((d > 0.5*camera_factor) && (d < 1.2*camera_factor))
        {
          //���������Ŀռ�����
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
  // //���ӻ����ͼ
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
  
  ros::Rate r(100); //100Hz,���ȴ�10ms 
  //�ж����ͼ�Ƿ����ڸ���
  while(pose_est_.bUpdatingImage)
  {
    ROS_INFO("Waiting for saving depth image");
    ros::spinOnce();
    r.sleep();
  }
  //�ر�������ͼ���ģ�1������׼�����������ͼ��2��������׼���̶���ͼ�����Դ������
  //depth_sub.shutdown();
  this->depth_sub_.shutdown();
  //���ͼ��mask����׼������������ƥ��
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

    //�����任�������ģ��
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
  
  //�ָ�����������ͼ
  //depth_sub_ = node_handle.subscribe("/camera/depth_registered/sw_registered/image_rect_raw", 1 , &Listener::Depth_Callback, this);
  depth_sub_ = node_handle.subscribe("/camera/depth/raw_image", 1 , &Listener::Depth_Callback, this);
}

//���ĵ��Ƽ����ӻ�
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
    
//     //�ָ���Ʋ���
//     pcl::VoxelGrid<PointT> grid; 
// 		grid.setLeafSize (0.001, 0.001, 0.001); //������Ԫ�����Ҷ�Ӵ�С
// 		//�²��� Դ���� 
// 		grid.setInputCloud (cloud.makeShared()); //�����������
// 		grid.filter (CloudAfterSample); //�²������˲������洢��src��

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


  
