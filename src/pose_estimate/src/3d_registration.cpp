
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
    grid.setLeafSize (0.001, 0.001, 0.001); //设置体元网格的叶子大小
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
		pcl::ScopeTime scope_time("compute_feature");//计算算法运行时间
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
    pcl_v_.p->removePointCloud ("source_cloud_v1"); //根据给定的ID，从屏幕中去除一个点云。参数是ID
    pcl_v_.p->removePointCloud ("target_cloud_v1");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(source_filtered, 250, 0, 0);
    pcl_v_.p->addPointCloud(source_filtered, sources_cloud_color, "source_cloud_v1", pcl_v_.vp_1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(target_filtered, 0, 250, 0);
    pcl_v_.p->addPointCloud(target_filtered, target_cloud_color, "target_cloud_v1", pcl_v_.vp_1);
    pcl_v_.p->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source_cloud_v1");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>aligend_cloud_color(transformed_cloud, 255, 0, 0);
    pcl_v_.p->addPointCloud(transformed_cloud, aligend_cloud_color, "aligend_cloud_v2", pcl_v_.vp_2);
    pcl_v_.p->addPointCloud(target_filtered, target_cloud_color, "target_cloud_v2", pcl_v_.vp_2);
    pcl_v_.p->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligend_cloud_v2");
    pcl_v_.p->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v2");

    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;
    boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
    crude_cor_est.setInputSource(source_fpfh);
    crude_cor_est.setInputTarget(target_fpfh);
    crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences);
    cout << "crude size is:" << cru_correspondences->size() << endl;
    pcl_v_.p->addCorrespondences<pcl::PointXYZ>(source_filtered, target_filtered, *cru_correspondences, "correspondence", pcl_v_.vp_1);//添加显示对应点对

    PCL_INFO ("Press q to continue ICP.\n");
    pcl_v_.p->spin();

    pcl_v_.p->removePointCloud("aligend_cloud_v2");
    pcl_v_.p->removePointCloud("target_cloud_v2");
  }
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
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0); //设置点云显示颜色，下同
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
    pcl_v_.p->addPointCloud (output, cloud_tgt_h, "target", pcl_v_.vp_2); //添加点云数据，下同
    pcl_v_.p->addPointCloud (cloud_src, cloud_src_h, "source", pcl_v_.vp_2);

    PCL_INFO ("Press q to clear the screen.\n");
    pcl_v_.p->spin ();

    pcl_v_.p->removeAllShapes(); 
    pcl_v_.p->removePointCloud ("source"); //根据给定的ID，从屏幕中去除一个点云。参数是ID
    pcl_v_.p->removePointCloud ("target"); //根据给定的ID，从屏幕中去除一个点云。参数是ID
  }
}
