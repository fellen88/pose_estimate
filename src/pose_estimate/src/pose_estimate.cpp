#include "pose_estimate.h"

pose_estimate::pose_estimate(const ros::NodeHandle& nodehandle, bool DebugVisualizer):
               rviz_v_(nodehandle),
               registration_(DebugVisualizer),
               CloudMask(new pcl::PointCloud<pcl::PointXYZ>),
               CloudModel (new pcl::PointCloud<pcl::PointXYZ>),
               CloudModelAfterSample (new pcl::PointCloud<pcl::PointXYZ>),
               CloudPreProcess (new pcl::PointCloud<pcl::PointXYZ>),
               CloudEuclideanCluster(new pcl::PointCloud<pcl::PointXYZ>),
               CloudEuclideanClusterAfterSample(new pcl::PointCloud<pcl::PointXYZ>),
               CloudTransformedTarget(new pcl::PointCloud<pcl::PointXYZ>)
{
  label = "bottle_milktea";
  depth_cols = 2064;
  depth_rows = 1544;
  camera_factor = 1000;
  camera_cx = 310.535;
  camera_cy = 239.405;
  camera_fx = 515.73;
  camera_fy = 515.45;

  thetax = 0;
  thetay = 0;
  thetaz = 0;

  Rotation_matrix = Eigen::Matrix3d::Identity();
  GlobalTransformation = Eigen::Matrix4f::Identity();
  UpsideDownTransformation = Eigen::Matrix4f::Identity();
  GlobalTransformationForGrasp = Eigen::Matrix4f::Identity();
  bSaveImage = false;
  bSaveCloud = false;
  DEBUG_VISUALIZER = DebugVisualizer;
}

int pose_estimate::segmentation()
{
  ROS_INFO("start segmentation");
  for(int ImgWidth = 0; ImgWidth < depth_rows; ImgWidth++)
  {
    for(int ImgHeight = 0; ImgHeight < depth_cols; ImgHeight++ )
    {
      //获取深度图中对应点的深度值
      float d = depth_ptr->image.at<float>(ImgWidth,ImgHeight);
      d = d / 1000;

      //有效范围内的点
      //if((d > 0.4*camera_factor) && (d < 2*camera_factor))
      {
		  //判断mask中是否是物体的点
		  if(mask_ptr != 0)
		  {
		    unsigned char t = mask_ptr->image.at<unsigned char>(ImgWidth,ImgHeight);
        ROS_INFO("d = %d", t);
		    if(t == 0)
		    continue;
		  }
		  else
		  {
		    ROS_INFO("mask image pointer mask_ptr = null");
		    continue;
		  }
		  //计算这个点的空间坐标
		  pcl::PointXYZ PointWorld;
		  PointWorld.z = double(d)/camera_factor;
		  PointWorld.x = (ImgHeight - camera_cx)*PointWorld.z/camera_fx;
		  PointWorld.y = (ImgWidth - camera_cy)*PointWorld.z/camera_fy;
		  CloudMask->points.push_back(PointWorld);
      }
    }
  }
  //设置点云属性，采用无序排列方式存储点云
  CloudMask->height = 1;
  CloudMask->width = CloudMask->points.size();
  ROS_INFO("mask cloud size = %d", CloudMask->width);
  if(0 == CloudMask->points.size())
  {
    ROS_INFO("Mask points number = 0 !!!");
    return 0;
  }
  //去除NaN点
  std::vector<int> nan_indices;
  pcl::removeNaNFromPointCloud(*CloudMask, *CloudMask, nan_indices);
  CloudMask->is_dense = false;
  if(true == DEBUG_VISUALIZER)
  {
    registration_.pcl_v_.p->removePointCloud ("target"); 
    registration_.pcl_v_.p->removePointCloud ("source");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> Mask_h (CloudMask, 255, 255, 255);
    registration_.pcl_v_.p->addPointCloud (CloudMask, Mask_h, "cloud_mask", registration_.pcl_v_.vp_1);
    //p->addPointCloud<pcl::PointXYZ>(CloudMask, "cloud mask");
    //p->spin();
  }
  return CloudMask->points.size();
}

int pose_estimate::EuclideanCluster(const PointCloud::Ptr cloud_Segmentation, const PointCloud::Ptr cloud_EuclideanCluster)
{
  //********************************欧式聚类******************************//
  try
  {
    //创建用于提取搜索方法的kdtree树对象 
    ROS_INFO("Start EuclideanCluster");

      //  for (int i = 0; i < cloud_Segmentation->points.size(); ++i)
    //   {
    //     printf("x = %f", cloud_Segmentation->points[i].x);
    //      printf("y = %f", cloud_Segmentation->points[i].y);
    //       printf("z = %f \n", cloud_Segmentation->points[i].z);
        
    //   }
    //   ROS_INFO("print end");

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_Segmentation);
    // ROS_INFO("set input end");

    std::vector<pcl::PointIndices> cluster_indices;
    //  printf("indices = %d", cluster_indices.size());
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; //欧式聚类对象
    ec.setClusterTolerance(0.01);                      //设置近邻搜索的搜索半径为2cm
    ec.setMinClusterSize(1);                         //设置一个聚类需要的最少的点数目为10
    ec.setMaxClusterSize(1000000);                     //设置一个聚类需要的最大点数目为250000
    ec.setSearchMethod(tree);                          //设置点云的搜索机制
    ec.setInputCloud(cloud_Segmentation);              //输入点云
    ec.extract(cluster_indices);                        //从点云中提取聚类，并将点云索引保存在cluster_indices中
    // ROS_INFO("extract end");               

    //容器中的点云的索引第一个为物体点云数据
    //if(cluster_indices > 0)
    //printf("indices = %d", cluster_indices.size());
    std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
   
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      cloud_EuclideanCluster->points.push_back(cloud_Segmentation->points[*pit]);
    }
    //设置保存点云的属性问题
    cloud_EuclideanCluster->width = cloud_EuclideanCluster->points.size();
    cloud_EuclideanCluster->height = 1;
    cloud_EuclideanCluster->is_dense = true;
    //std::cout << "PointCloud representing the Cluster: " << cloud_EuclideanCluster->points.size() << " data points." << std::endl;
    ROS_INFO("End EuclideanCluster");
  }
  catch(int e)
  {
    //cout << e.what() << endl;
    PCL_INFO ("Points can't be used for EuclideanCluster !");
    printf("******************************************************");
    printf("\n");
    printf("******************************************************");
    printf("\n");
    printf("******************************************************");
    printf("\n");
    return -1;
  }     
  return 0;
}

//整合的配准函数
int pose_estimate::Alignment()
{
  float AngleObjectZtoCameraZ = 0;
  { //计算算法运行时间
    pcl::ScopeTime scope_time("**TotalAlignment");
    //利用深度图和mask分割场景中物体点云 
    if(segmentation() < 50)
    {
      CloudMask->points.clear();
      printf("******************************************************");
      printf("\n");
      return 2;
    }
    //欧式聚类去处理离群点，保留最大点集，避免RGB—D对齐误差或者MaskRCNN识别误差导致分层现象
    if(-1 == EuclideanCluster(CloudMask, CloudEuclideanCluster))
    {
       ROS_INFO("EuclideanCluster can't work !");
       printf("******************************************************");
       printf("\n");
       return 3;
    }
    
    //清空CloudMask
    CloudMask->points.clear();
    if(true == DEBUG_VISUALIZER)
    {
      registration_.pcl_v_.p->addCoordinateSystem(0.2);
      //p->removePointCloud ("cloud mask");
      //p->addPointCloud<pcl::PointXYZ>(CloudEuclideanCluster, "cloud EuclideanCluster");
      //p->spin();
    }
 
    //根据label提取物体模型
    std::string ModelPath = "/home/siasuncv/RobGrab/src/pose_estimate/3Dmodels/";
    //std::string ModelPath = "/home/model/catkin_ws2/src/pose_estimation/3Dmodels/";
    ModelPath = ModelPath + label + "_model.pcd";
     //std::cout << "ModelPath : " << ModelPath << endl;
     int pointRatio = 0;
     CloudModel->points.clear();
     CloudModelAfterSample->points.clear();
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (ModelPath, *CloudModel) == -1) 
    {
      //PCL_ERROR ("Couldn't read file bottle_milktea_model.pcd \n");
      std::cout << "Couldn't read file " << ModelPath <<endl;
      printf("******************************************************");
      printf("\n");
      return -1;
    }
 
    else
    {
		pcl::VoxelGrid<PointT> grid; 
		grid.setLeafSize (0.001, 0.001, 0.001); //设置体元网格的叶子大小
		//下采样 源点云 
		grid.setInputCloud (CloudEuclideanCluster); //设置输入点云
		grid.filter (*CloudEuclideanClusterAfterSample); //下采样和滤波，并存储在src中
		//下采样 目标点云
		grid.setInputCloud (CloudModel);
		grid.filter (*CloudModelAfterSample);
		cout <<  CloudEuclideanClusterAfterSample->points.size()<<endl;
		cout << CloudModelAfterSample->points.size()<<endl;
		pointRatio = 100*(CloudEuclideanClusterAfterSample->points.size())/CloudModelAfterSample->points.size();
    ROS_INFO("CloudMask/CloudModel = %d !", pointRatio);
    // if(pointRatio > 60)
    //   {
    //     CloudEuclideanCluster->points.clear();
    //     CloudEuclideanClusterAfterSample->points.clear();
    //     ROS_INFO("CloudMask/CloudModel > 0.6 !");
    //     printf("******************************************************");
    //     printf("\n");
    //     return 1;
    //   }
    //   else if(pointRatio < 20)
    //   {
    //     CloudEuclideanCluster->points.clear();
    //     CloudEuclideanClusterAfterSample->points.clear();
    //     ROS_INFO("CloudMask/CloudModel < 0.2 !");
    //     printf("******************************************************");
    //     printf("\n");
    //     return 2;
    //   }
     }    
    //cout << "points loaded from Model =" << CloudModel->width * CloudModel->height <<endl;
   
    if(true == DEBUG_VISUALIZER)
    {
      registration_.pcl_v_.showCloudsLeft(CloudModel, CloudEuclideanCluster); //在左视区，显示源点云和目标点云
      //showCloudsRight(CloudEuclideanClusterAfterSample, CloudModel);//在右视区，显示源点云和目标点云
    }     
    //配准物体模型和场景中物体点云
    ROS_INFO("start Alignment");
    {
      pcl::ScopeTime scope_time("*PrePairAlign");//计算算法运行时间
      //AngleObjectZtoCameraZ = registration_.prePairAlign(CloudEuclideanCluster,CloudModel,CloudPreProcess,true);
      registration_.prePairAlign(CloudEuclideanCluster,CloudModel,CloudPreProcess, Pre_PairAlign_Transformation, true);
      registration_.SAC_IA_PareAlign(CloudEuclideanCluster,CloudModel,CloudPreProcess, Pre_PairAlign_Transformation, true);

    }
    //ICP匹配
    {
      pcl::ScopeTime scope_time("*PairAlign");//计算算法运行时间
      registration_.pairAlign (CloudEuclideanCluster, CloudPreProcess, CloudTransformedTarget, PairAlign_Transformation, true);

      bSaveImage = false;
      bSaveCloud = false;
    }
  } //结束计算算法时间
  CloudEuclideanCluster->points.clear();
  CloudEuclideanClusterAfterSample->points.clear();
  if(true == DEBUG_VISUALIZER)
  {
    PCL_INFO ("Press q to finish.\n");
    registration_.pcl_v_.p->spin();
  }
  
  printf("\n");
  std::cout << "The Estimated Rotation and translation matrices are : \n" << std::endl;
  printf("\n");
  printf("    | %6.3f %6.3f %6.3f | \n", PairAlign_Transformation(0, 0), PairAlign_Transformation(0, 1), PairAlign_Transformation(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", PairAlign_Transformation(1, 0), PairAlign_Transformation(1, 1), PairAlign_Transformation(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", PairAlign_Transformation(2, 0), PairAlign_Transformation(2, 1), PairAlign_Transformation(2, 2));
  printf("\n");
  printf("t = < %0.3f, %0.3f, %0.3f >\n", PairAlign_Transformation(0, 3), PairAlign_Transformation(1, 3), PairAlign_Transformation(2, 3));
  printf("\n");

  GlobalTransformation = PairAlign_Transformation*registration_.MomentOfInertia_Transformation;
  std::cout << "The Global Rotation and translation matrices are : \n" << std::endl;
  printf("\n");
  printf("    | %6.3f %6.3f %6.3f | \n", GlobalTransformation(0, 0), GlobalTransformation(0, 1), GlobalTransformation(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", GlobalTransformation(1, 0), GlobalTransformation(1, 1), GlobalTransformation(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", GlobalTransformation(2, 0), GlobalTransformation(2, 1), GlobalTransformation(2, 2));
  printf("\n");
  printf("t = < %0.3f, %0.3f, %0.3f >\n", GlobalTransformation(0, 3), GlobalTransformation(1, 3), GlobalTransformation(2, 3));
  printf("******************************************************");
  printf("\n");

  // UpsideDownTransformation(0, 0) = 1;
  // UpsideDownTransformation(1, 1) = -1;
  // UpsideDownTransformation(2, 2) = -1;
  // UpsideDownTransformation(3, 3) = 1;
  // if(AngleObjectZtoCameraZ > 90)
  // {
  //     ROS_INFO("AngleObjectZtoCameraZ > 90 , convert!");
  //     GlobalTransformationForGrasp = GlobalTransformation*UpsideDownTransformation;
  //     //GlobalTransformationForGrasp = GlobalTransformation;GlobalTransformation.inverse()*
  //     //GlobalTransformationForGrasp = GlobalTransformation;

  //     std::cout << "The Global Rotation and translation matrices are : \n" << std::endl;
  //     printf("\n");
  //     printf("    | %6.3f %6.3f %6.3f %6.3f | \n", GlobalTransformationForGrasp(0, 0), GlobalTransformationForGrasp(0, 1), GlobalTransformationForGrasp(0, 2), GlobalTransformationForGrasp(0, 3));
  //     printf("R = | %6.3f %6.3f %6.3f %6.3f | \n", GlobalTransformationForGrasp(1, 0), GlobalTransformationForGrasp(1, 1), GlobalTransformationForGrasp(1, 2), GlobalTransformationForGrasp(1, 3));
  //     printf("    | %6.3f %6.3f %6.3f %6.3f | \n", GlobalTransformationForGrasp(2, 0), GlobalTransformationForGrasp(2, 1), GlobalTransformationForGrasp(2, 2), GlobalTransformationForGrasp(2, 3));
  //     printf("    | %6.3f %6.3f %6.3f %6.3f | \n", GlobalTransformationForGrasp(3, 0), GlobalTransformationForGrasp(3, 1), GlobalTransformationForGrasp(3, 2), GlobalTransformationForGrasp(3, 3));
  //     printf("\n");
  //     printf("******************************************************");
  // }
  // else
  {
    GlobalTransformationForGrasp = GlobalTransformation;
  }
  udateTF();
  rviz_v_.updateBoundingBox(CloudTransformedTarget, true);
  return 0;
}

void pose_estimate::udateTF()
{
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
    {
      //Rotation_matrix(i, j) = GlobalTransformation(i, j);
      //物体姿态z方向向上，绕x轴旋转180度
      Rotation_matrix(i, j) = GlobalTransformationForGrasp(i, j);
    }
  euler_Angle = Rotation_matrix.eulerAngles(2, 1, 0);//顺序Z, Y, X
  thetax = euler_Angle[2];
  thetay = euler_Angle[1];
  thetaz = euler_Angle[0];
}

