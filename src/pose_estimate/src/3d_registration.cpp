
#include "3d_registration.h"
#include <math.h>

registration::registration(bool DebugVisualizer): pcl_v_(DebugVisualizer)
{
  DEBUG_VISUALIZER = DebugVisualizer;
}

//为了使用fpfp特征匹配，声明一个计算fpfh特征点的函数：
fpfhFeature::Ptr registration::compute_fpfh_feature(PointCloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
	//法向量
	pointnormal::Ptr point_normal(new pointnormal);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
	est_normal.setInputCloud(input_cloud);
	est_normal.setSearchMethod(tree);
	est_normal.setKSearch(20);
	est_normal.compute(*point_normal);
	//fpfh 估计
	fpfhFeature::Ptr fpfh(new fpfhFeature);
	//pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_fpfh;
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(1); //指定4核计算
									// pcl::search::KdTree<pcl::PointXYZ>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZ> ());
	est_fpfh.setInputCloud(input_cloud);
	est_fpfh.setInputNormals(point_normal);
	est_fpfh.setSearchMethod(tree);
	est_fpfh.setKSearch(20);
	est_fpfh.compute(*fpfh);

	return fpfh;
}

void registration::SAC_IA_PareAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr transformed_cloud, Eigen::Matrix4f &SAC_transform, bool downsample)
{
  //为了一致性和速度，下采样
  PointCloud::Ptr source_filtered(new PointCloud); //创建点云指针
  PointCloud::Ptr target_filtered(new PointCloud);
  pcl::VoxelGrid<PointT> grid; //VoxelGrid 把一个给定的点云，聚集在一个局部的3D网格上,并下采样和滤波点云数据
  if (downsample) //下采样
  {
    grid.setLeafSize (0.005, 0.005, 0.005); //设置体元网格的叶子大小
        //下采样 源点云
    grid.setInputCloud (cloud_src); //设置输入点云
    grid.filter (*source_filtered); //下采样和滤波，并存储在src中
        //下采样 目标点云
    grid.setInputCloud (cloud_tgt);
    grid.filter (*target_filtered);
  }
  else //不下采样
  {
    source_filtered = cloud_src; //直接复制
    target_filtered = cloud_tgt;
  }

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	fpfhFeature::Ptr source_fpfh;
	fpfhFeature::Ptr target_fpfh;
	{
		pcl::ScopeTime scope_time("*PrePairAlign");//计算算法运行时间
		source_fpfh = compute_fpfh_feature(source_filtered, tree);
		target_fpfh = compute_fpfh_feature(target_filtered, tree);
	}

	//对齐(占用了大部分运行时间)
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
	sac_ia.setInputSource(source_filtered);
	sac_ia.setSourceFeatures(source_fpfh);
	sac_ia.setInputTarget(target_filtered);
	sac_ia.setTargetFeatures(target_fpfh);
	//sac_ia.setNumberOfSamples(20);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
	sac_ia.setCorrespondenceRandomness(10); //设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
	sac_ia.align(*transformed_cloud);

	SAC_transform = sac_ia.getFinalTransformation();

	//可视化
	if (true == DEBUG_VISUALIZER)
	{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("fpfh test"));
	int v1;
	int v2;

	view->createViewPort(0, 0.0, 0.5, 1.0, v1);
	view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	view->setBackgroundColor(0, 0, 0, v1);
	view->setBackgroundColor(0, 0, 0, v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(source_filtered, 250, 0, 0);
	view->addPointCloud(source_filtered, sources_cloud_color, "sources_cloud_v1", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(target_filtered, 0, 250, 0);
	view->addPointCloud(target_filtered, target_cloud_color, "target_cloud_v1", v1);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sources_cloud_v1");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>aligend_cloud_color(transformed_cloud, 255, 0, 0);
	view->addPointCloud(transformed_cloud, aligend_cloud_color, "aligend_cloud_v2", v2);

	view->addPointCloud(target_filtered, target_cloud_color, "target_cloud_v2", v2);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligend_cloud_v2");
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v2");

	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;

	boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
	crude_cor_est.setInputSource(source_fpfh);
	crude_cor_est.setInputTarget(target_fpfh);
	//  crude_cor_est.determineCorrespondences(cru_correspondences);
	crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences);
	cout << "crude size is:" << cru_correspondences->size() << endl;
	view->addCorrespondences<pcl::PointXYZ>(source_filtered, target_filtered, *cru_correspondences, "correspondence", v1);//添加显示对应点对

	view->spin();
	}
}

void registration::prePairAlign(const PointCloud::Ptr cloud_src,const PointCloud::Ptr cloud_tgt, PointCloud::Ptr transformed_cloud, Eigen::Matrix4f &pre_transform,bool downsample)
{
  PointCloud::Ptr src (new PointCloud); //创建点云指针
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid; //VoxelGrid 把一个给定的点云，聚集在一个局部的3D网格上,并下采样和滤波点云数据
  if (downsample) //下采样
  {
    grid.setLeafSize (0.001, 0.001, 0.001); //设置体元网格的叶子大小
        //下采样 源点云
    grid.setInputCloud (cloud_src); //设置输入点云
    grid.filter (*src); //下采样和滤波，并存储在src中
        //下采样 目标点云
    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);

    //PCL_INFO ("Partial Pointcloud size after sampling is. %d.\n", src->size());
    //PCL_INFO ("Model Pointcloud size after sampling is. %d.\n", tgt->size());
  }
  else //不进行下采样
  {
    src = cloud_src; //直接复制
    tgt = cloud_tgt;
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

  //***********************************可视化重心、包围盒和坐标系******************************************//
  if(true == DEBUG_VISUALIZER)
  {
    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    //pcl_v_.p->setBackgroundColor(0, 0, 0);
    //pcl_v_.p->setCameraPose();
    //pcl_v_.p->initCameraParameters();
    //pcl_v_.p->addPointCloud<pcl::PointXYZ>(cloud_src, "prePairAlign source");
    //pcl_v_.p->addPointCloud<pcl::PointXYZ>(cloud_tgt, "prePairAlign target");
    //pcl_v_.p->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
  }

	Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
	Eigen::Quaternionf quat(rotational_matrix_OBB);
	//pcl_v_.p->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");

	pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
	pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
	pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
	pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
  //if(true == DEBUG_VISUALIZER)
  //{
  //  pcl_v_.p->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
  //  pcl_v_.p->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
  //  pcl_v_.p->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
  //}

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

	pcl::PointXYZ pt1(p1(0), p1(1), p1(2));
	pcl::PointXYZ pt2(p2(0), p2(1), p2(2));
	pcl::PointXYZ pt3(p3(0), p3(1), p3(2));
	pcl::PointXYZ pt4(p4(0), p4(1), p4(2));
	pcl::PointXYZ pt5(p5(0), p5(1), p5(2));
	pcl::PointXYZ pt6(p6(0), p6(1), p6(2));
	pcl::PointXYZ pt7(p7(0), p7(1), p7(2));
	pcl::PointXYZ pt8(p8(0), p8(1), p8(2));

  //if(true == DEBUG_VISUALIZER)
  //{
  //  pcl_v_.p->addLine(pt1, pt2, 1.0, 0.0, 0.0, "1 edge");
  //  pcl_v_.p->addLine(pt1, pt4, 1.0, 0.0, 0.0, "2 edge");
  //  pcl_v_.p->addLine(pt1, pt5, 1.0, 0.0, 0.0, "3 edge");
  //  pcl_v_.p->addLine(pt3, pt2, 1.0, 0.0, 0.0, "4 edge");
  //  pcl_v_.p->addLine(pt3, pt4, 1.0, 0.0, 0.0, "5 edge");
  //  pcl_v_.p->addLine(pt3, pt7, 1.0, 0.0, 0.0, "6 edge");
  //  pcl_v_.p->addLine(pt6, pt2, 1.0, 0.0, 0.0, "7 edge");
  //  pcl_v_.p->addLine(pt6, pt5, 1.0, 0.0, 0.0, "8 edge");
  //  pcl_v_.p->addLine(pt6, pt7, 1.0, 0.0, 0.0, "9 edge");
  //  pcl_v_.p->addLine(pt8, pt4, 1.0, 0.0, 0.0, "10 edge");
  //  pcl_v_.p->addLine(pt8, pt5, 1.0, 0.0, 0.0, "11 edge");
  //  pcl_v_.p->addLine(pt8, pt7, 1.0, 0.0, 0.0, "12 edge");
  //}

	//************************************计算旋转矩阵***********************************//
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());

	cloud_in->width = 4;
	cloud_in->height = 1;
	cloud_in->is_dense = false;
	cloud_in->resize(cloud_in->width * cloud_in->height);

	cloud_out->width = 4;
	cloud_out->height = 1;
	cloud_out->is_dense = false;
	cloud_out->resize(cloud_out->width * cloud_out->height);

	//输入四个点
	cloud_in->points[0].x = 0;
	cloud_in->points[0].y = 0;
	cloud_in->points[0].z = 0;

	cloud_in->points[1].x = 1;
	cloud_in->points[1].y = 0;
	cloud_in->points[1].z = 0;

	cloud_in->points[2].x = 0;
	cloud_in->points[2].y = 1;
	cloud_in->points[2].z = 0;

	cloud_in->points[3].x = 0;
	cloud_in->points[3].y = 0;
	cloud_in->points[3].z = 1;

	//目标四个点
	cloud_out->points[0].x = center.x;
	cloud_out->points[0].y = center.y;
	cloud_out->points[0].z = center.z;

	cloud_out->points[1].x = x_axis.x;
	cloud_out->points[1].y = x_axis.y;
	cloud_out->points[1].z = x_axis.z;

	cloud_out->points[2].x = y_axis.x;
	cloud_out->points[2].y = y_axis.y;
	cloud_out->points[2].z = y_axis.z;

	cloud_out->points[3].x = z_axis.x;
	cloud_out->points[3].y = z_axis.y;
	cloud_out->points[3].z = z_axis.z;

	//利用SVD方法求解变换矩阵  
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;

	TESVD.estimateRigidTransformation(*cloud_in, *cloud_out, MomentOfInertia_Transformation);
  //Executing the transformation
	//pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	//You can either apply transform_1 or transform_2; they are the same
  //pcl::transformPointCloud(*cloud_src, *transformed_cloud, MomentOfInertia_Transformation);
  pcl::transformPointCloud(*cloud_tgt, *transformed_cloud, MomentOfInertia_Transformation);
  pre_transform = MomentOfInertia_Transformation;
  // if(true == DEBUG_VISUALIZER)
  //{
  //  pcl_v_.p->addPointCloud<pcl::PointXYZ>(transformed_cloud, "prePairAlign cloud");
  //}
  //pcl::io::savePCDFileASCII("transformed_cloud.pcd", *transformed_cloud);	

	//输出变换矩阵信息  
	std::cout << "The Pre-estimated Rotation and translation matrices are : \n" << std::endl;
	printf("\n");
	printf("    | %6.3f %6.3f %6.3f | \n", MomentOfInertia_Transformation(0, 0), MomentOfInertia_Transformation(0, 1), MomentOfInertia_Transformation(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", MomentOfInertia_Transformation(1, 0), MomentOfInertia_Transformation(1, 1), MomentOfInertia_Transformation(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", MomentOfInertia_Transformation(2, 0), MomentOfInertia_Transformation(2, 1), MomentOfInertia_Transformation(2, 2));
	printf("\n");
	printf("t = < %0.3f, %0.3f, %0.3f >\n", MomentOfInertia_Transformation(0, 3), MomentOfInertia_Transformation(1, 3), MomentOfInertia_Transformation(2, 3));
  printf("\n");

  //物体姿态z方向向上的情况，传出标志，对最终变换矩阵绕物体x轴旋转180度
 // Eigen::Vector3f ObjectZ(z_axis.x - center.x, z_axis.y - center.y, z_axis.z - center.z);
 // float NormOfObjectZ = sqrt(ObjectZ(0)*ObjectZ(0) + ObjectZ(1)*ObjectZ(1) + ObjectZ(2)*ObjectZ(2));
 // Eigen::Vector3f CameraZ(0.0, 0.0, 1.0);
 // float NormOfCameraZ = sqrt(CameraZ(0)*CameraZ(0) +  CameraZ(1)* CameraZ(1) +  CameraZ(2)* CameraZ(2));
 // float Theta = 180*acos((ObjectZ.dot(CameraZ))/(NormOfObjectZ*NormOfCameraZ))/3.14159;
 // cout <<NormOfObjectZ   <<"," <<NormOfCameraZ << endl;
 // ROS_INFO("Angle between ObjectZ and CameraZ is %f", Theta);
 // return Theta;
}

void registration::pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  //为了一致性和速度，下采样
  PointCloud::Ptr src (new PointCloud); //创建点云指针
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid; //VoxelGrid 把一个给定的点云，聚集在一个局部的3D网格上,并下采样和滤波点云数据
  if (downsample) //下采样
  {
    grid.setLeafSize (0.001, 0.001, 0.001); //设置体元网格的叶子大小
        //下采样 源点云
    grid.setInputCloud (cloud_src); //设置输入点云
    grid.filter (*src); //下采样和滤波，并存储在src中
        //下采样 目标点云
    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else //不下采样
  {
    src = cloud_src; //直接复制
    tgt = cloud_tgt;
  }

  //计算曲面的法向量和曲率
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals); //创建源点云指针（注意点的类型包含坐标和法向量）
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals); //创建目标点云指针（注意点的类型包含坐标和法向量）
  pcl::NormalEstimation<PointT, PointNormalT> norm_est; //该对象用于计算法向量
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ()); //创建kd树，用于计算法向量的搜索方法
  norm_est.setSearchMethod (tree); //设置搜索方法
  norm_est.setKSearch (30); //设置最近邻的数量
  norm_est.setInputCloud (src); //设置输入云
  norm_est.compute (*points_with_normals_src); //计算法向量，并存储在points_with_normals_src
  pcl::copyPointCloud (*src, *points_with_normals_src); //复制点云（坐标）到points_with_normals_src（包含坐标和法向量）
  norm_est.setInputCloud (tgt); //这3行计算目标点云的法向量，同上
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //创建一个 自定义点表达方式的 实例
  MyPointRepresentation point_representation;
  //加权曲率维度，以和坐标xyz保持平衡
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha); //设置缩放值（向量化点时使用）

  //创建非线性ICP对象 并设置参数
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg; //创建非线性ICP对象（ICP变体，使用Levenberg-Marquardt最优化）
  reg.setTransformationEpsilon (1e-6); //设置容许的最大误差（迭代最优化）
  //reg.setTransformationEpsilon (0.01); //设置容许的最大误差（迭代最优化）
  //***** 注意：根据自己数据库的大小调节该参数
  reg.setMaxCorrespondenceDistance (2);  //设置对应点之间的最大距离（2m）,在配准过程中，忽略大于该阈值的点  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation)); //设置点表达
  //设置源点云和目标点云
  reg.setInputSource (points_with_normals_src); //版本不符合，使用下面的语句
  //reg.setInputCloud (points_with_normals_src); //设置输入点云（待变换的点云）
  reg.setInputTarget (points_with_normals_tgt); //设置目标点云
  reg.setMaximumIterations (2); //设置内部优化的迭代次数

  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src; //用于存储结果（坐标+法向量）

  int NumIteration = 0;
  for (int i = 0; i < 200; ++i) //迭代
  {
    //pcl::ScopeTime scope_time("ICP Iteration"); 
    //PCL_INFO ("Iteration Nr. %d.\n", i); //命令行显示迭代的次数
    //保存点云，用于可视化
    points_with_normals_src = reg_result; //
    //估计
    reg.setInputSource (points_with_normals_src);
    //reg.setInputCloud (points_with_normals_src); //重新设置输入点云（待变换的点云），因为经过上一次迭代，已经发生变换了
    reg.align (*reg_result); //对齐（配准）两个点云

    Ti = reg.getFinalTransformation () * Ti; //累积（每次迭代的）变换矩阵
    //如果这次变换和上次变换的误差比阈值小，通过减小最大的对应点距离的方法来进一步细化
    // if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
    //    break;
       
          //如果这次变换和上次变换的误差比阈值小，通过减小最大的对应点距离的方法来进一步细化
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001); //减小对应点之间的最大距离（上面设置过）
     prev = reg.getLastIncrementalTransformation (); //上一次变换的误差
    //std::cout<<"getLastIncrementalTransformation"<<reg.getLastIncrementalTransformation ()<<endl;
    //std::cout<<"getLastIncrementalTransformation.sum: "<<reg.getLastIncrementalTransformation ().sum()<<endl;
    NumIteration = i;
    //显示当前配准状态，在窗口的右视区，简单的显示源点云和目标点云
    if(true == DEBUG_VISUALIZER)
    {
      pcl_v_.showCloudsRight(points_with_normals_tgt, points_with_normals_src);    
    }
  }

 PCL_INFO ("Iteration Nr. %d.\n", NumIteration); //命令行显示迭代的次数
  targetToSource = Ti.inverse(); //计算从目标点云到源点云的变换矩阵
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource); //将目标点云 变换回到 源点云帧

  //add the source to the transformed target
  //*output += *cloud_src; // 拼接点云图（的点）点数数目是两个点云的点数和
  final_transform = targetToSource; //最终的变换。目标点云到源点云的变换矩阵

  if(true == DEBUG_VISUALIZER)
  {
    pcl_v_.p->removePointCloud ("source"); //根据给定的ID，从屏幕中去除一个点云。参数是ID
    pcl_v_.p->removePointCloud ("target");
    pcl_v_.p->removePointCloud("prePairAlign cloud");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0); //设置点云显示颜色，下同
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
    pcl_v_.p->addPointCloud (output, cloud_tgt_h, "target", pcl_v_.vp_2); //添加点云数据，下同
    pcl_v_.p->addPointCloud (cloud_src, cloud_src_h, "source", pcl_v_.vp_2);

    PCL_INFO ("Press q to clear the screen.\n");
    pcl_v_.p->spin ();

    pcl_v_.p->removePointCloud ("prePairAlign source"); //根据给定的ID，从屏幕中去除一个点云。参数是ID
    pcl_v_.p->removePointCloud ("prePairAlign target"); //根据给定的ID，从屏幕中去除一个点云。参数是ID
    pcl_v_.p->removeAllShapes();
    
    pcl_v_.p->removePointCloud ("source"); //根据给定的ID，从屏幕中去除一个点云。参数是ID
    pcl_v_.p->removePointCloud ("target");
    pcl_v_.p->removePointCloud ("cloud_mask");
    pcl_v_.p->removePointCloud ("vp1_target"); //根据给定的ID，从屏幕中去除一个点云。参数是ID
    pcl_v_.p->removePointCloud ("vp1_source");
  }
}
