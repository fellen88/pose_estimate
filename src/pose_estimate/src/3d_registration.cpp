
#include "3d_registration.h"
#include <math.h>

registration::registration(bool DebugVisualizer): pcl_v_(DebugVisualizer)
{
  DEBUG_VISUALIZER = DebugVisualizer;
}

//Ϊ��ʹ��fpfp����ƥ�䣬����һ������fpfh������ĺ�����
fpfhFeature::Ptr registration::compute_fpfh_feature(PointCloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
	//������
	pointnormal::Ptr point_normal(new pointnormal);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
	est_normal.setInputCloud(input_cloud);
	est_normal.setSearchMethod(tree);
	est_normal.setKSearch(20);
	est_normal.compute(*point_normal);
	//fpfh ����
	fpfhFeature::Ptr fpfh(new fpfhFeature);
	//pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_fpfh;
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(1); //ָ��4�˼���
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
  //Ϊ��һ���Ժ��ٶȣ��²���
  PointCloud::Ptr source_filtered(new PointCloud); //��������ָ��
  PointCloud::Ptr target_filtered(new PointCloud);
  pcl::VoxelGrid<PointT> grid; //VoxelGrid ��һ�������ĵ��ƣ��ۼ���һ���ֲ���3D������,���²������˲���������
  if (downsample) //�²���
  {
    grid.setLeafSize (0.001, 0.001, 0.001); //������Ԫ�����Ҷ�Ӵ�С
        //�²��� Դ����
    grid.setInputCloud (cloud_src); //�����������
    grid.filter (*source_filtered); //�²������˲������洢��src��
        //�²��� Ŀ�����
    grid.setInputCloud (cloud_tgt);
    grid.filter (*target_filtered);
  }
  else //���²���
  {
    source_filtered = cloud_src; //ֱ�Ӹ���
    target_filtered = cloud_tgt;
  }

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	fpfhFeature::Ptr source_fpfh;
	fpfhFeature::Ptr target_fpfh;
	{
		pcl::ScopeTime scope_time("compute_feature");//�����㷨����ʱ��
		source_fpfh = compute_fpfh_feature(source_filtered, tree);
		target_fpfh = compute_fpfh_feature(target_filtered, tree);
	}

	//����(ռ���˴󲿷�����ʱ��)
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
	sac_ia.setInputSource(source_filtered);
	sac_ia.setSourceFeatures(source_fpfh);
	sac_ia.setInputTarget(target_filtered);
	sac_ia.setTargetFeatures(target_fpfh);
	//sac_ia.setNumberOfSamples(20);  //����ÿ�ε���������ʹ�õ�������������ʡ��,�ɽ�ʡʱ��
	sac_ia.setCorrespondenceRandomness(10); //���ü���Э����ʱѡ����ٽ��ڵ㣬��ֵԽ��Э����Խ��ȷ�����Ǽ���Ч��Խ��.(��ʡ)
	sac_ia.align(*transformed_cloud);

	SAC_transform = sac_ia.getFinalTransformation();

	//���ӻ�

	if (true == DEBUG_VISUALIZER)
	{
    pcl_v_.p->removePointCloud ("source_cloud_v1"); //���ݸ�����ID������Ļ��ȥ��һ�����ơ�������ID
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
    pcl_v_.p->addCorrespondences<pcl::PointXYZ>(source_filtered, target_filtered, *cru_correspondences, "correspondence", pcl_v_.vp_1);//�����ʾ��Ӧ���

    PCL_INFO ("Press q to continue ICP.\n");
    pcl_v_.p->spin();

    pcl_v_.p->removePointCloud("aligend_cloud_v2");
    pcl_v_.p->removePointCloud("target_cloud_v2");
  }
}

void registration::pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  //Ϊ��һ���Ժ��ٶȣ��²���
  PointCloud::Ptr src (new PointCloud); //��������ָ��
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid; //VoxelGrid ��һ�������ĵ��ƣ��ۼ���һ���ֲ���3D������,���²������˲���������
  if (downsample) //�²���
  {
    grid.setLeafSize (0.001, 0.001, 0.001); //������Ԫ�����Ҷ�Ӵ�С
        //�²��� Դ����
    grid.setInputCloud (cloud_src); //�����������
    grid.filter (*src); //�²������˲������洢��src��
        //�²��� Ŀ�����
    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else //���²���
  {
    src = cloud_src; //ֱ�Ӹ���
    tgt = cloud_tgt;
  }

  //��������ķ�����������
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals); //����Դ����ָ�루ע�������Ͱ�������ͷ�������
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals); //����Ŀ�����ָ�루ע�������Ͱ�������ͷ�������
  pcl::NormalEstimation<PointT, PointNormalT> norm_est; //�ö������ڼ��㷨����
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ()); //����kd�������ڼ��㷨��������������
  norm_est.setSearchMethod (tree); //������������
  norm_est.setKSearch (30); //��������ڵ�����
  norm_est.setInputCloud (src); //����������
  norm_est.compute (*points_with_normals_src); //���㷨���������洢��points_with_normals_src
  pcl::copyPointCloud (*src, *points_with_normals_src); //���Ƶ��ƣ����꣩��points_with_normals_src����������ͷ�������
  norm_est.setInputCloud (tgt); //��3�м���Ŀ����Ƶķ�������ͬ��
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //����һ�� �Զ�����﷽ʽ�� ʵ��
  MyPointRepresentation point_representation;
  //��Ȩ����ά�ȣ��Ժ�����xyz����ƽ��
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha); //��������ֵ����������ʱʹ�ã�

  //����������ICP���� �����ò���
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg; //����������ICP����ICP���壬ʹ��Levenberg-Marquardt���Ż���
  reg.setTransformationEpsilon (1e-6); //���������������������Ż���
  //reg.setTransformationEpsilon (0.01); //���������������������Ż���
  //***** ע�⣺�����Լ����ݿ�Ĵ�С���ڸò���
  reg.setMaxCorrespondenceDistance (2);  //���ö�Ӧ��֮��������루2m��,����׼�����У����Դ��ڸ���ֵ�ĵ�  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation)); //���õ���
  //����Դ���ƺ�Ŀ�����
  reg.setInputSource (points_with_normals_src); //�汾�����ϣ�ʹ����������
  //reg.setInputCloud (points_with_normals_src); //����������ƣ����任�ĵ��ƣ�
  reg.setInputTarget (points_with_normals_tgt); //����Ŀ�����
  reg.setMaximumIterations (2); //�����ڲ��Ż��ĵ�������

  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src; //���ڴ洢���������+��������

  int NumIteration = 0;
  for (int i = 0; i < 200; ++i) //����
  {
    //pcl::ScopeTime scope_time("ICP Iteration"); 
    //PCL_INFO ("Iteration Nr. %d.\n", i); //��������ʾ�����Ĵ���
    //������ƣ����ڿ��ӻ�
    points_with_normals_src = reg_result; //
    //����
    reg.setInputSource (points_with_normals_src);
    //reg.setInputCloud (points_with_normals_src); //��������������ƣ����任�ĵ��ƣ�����Ϊ������һ�ε������Ѿ������任��
    reg.align (*reg_result); //���루��׼����������

    Ti = reg.getFinalTransformation () * Ti; //�ۻ���ÿ�ε����ģ��任����
    //�����α任���ϴα任��������ֵС��ͨ����С���Ķ�Ӧ�����ķ�������һ��ϸ��
    // if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
    //    break;
       
          //�����α任���ϴα任��������ֵС��ͨ����С���Ķ�Ӧ�����ķ�������һ��ϸ��
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001); //��С��Ӧ��֮��������루�������ù���
     prev = reg.getLastIncrementalTransformation (); //��һ�α任�����
    //std::cout<<"getLastIncrementalTransformation"<<reg.getLastIncrementalTransformation ()<<endl;
    //std::cout<<"getLastIncrementalTransformation.sum: "<<reg.getLastIncrementalTransformation ().sum()<<endl;
    NumIteration = i;
    //��ʾ��ǰ��׼״̬���ڴ��ڵ����������򵥵���ʾԴ���ƺ�Ŀ�����
    if(true == DEBUG_VISUALIZER)
    {
      pcl_v_.showCloudsRight(points_with_normals_tgt, points_with_normals_src);    
    }
  }

  PCL_INFO ("Iteration Nr. %d.\n", NumIteration); //��������ʾ�����Ĵ���
  targetToSource = Ti.inverse(); //�����Ŀ����Ƶ�Դ���Ƶı任����
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource); //��Ŀ����� �任�ص� Դ����֡

  //add the source to the transformed target
  //*output += *cloud_src; // ƴ�ӵ���ͼ���ĵ㣩������Ŀ���������Ƶĵ�����
  final_transform = targetToSource; //���յı任��Ŀ����Ƶ�Դ���Ƶı任����

  if(true == DEBUG_VISUALIZER)
  {
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0); //���õ�����ʾ��ɫ����ͬ
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
    pcl_v_.p->addPointCloud (output, cloud_tgt_h, "target", pcl_v_.vp_2); //��ӵ������ݣ���ͬ
    pcl_v_.p->addPointCloud (cloud_src, cloud_src_h, "source", pcl_v_.vp_2);

    PCL_INFO ("Press q to clear the screen.\n");
    pcl_v_.p->spin ();

    pcl_v_.p->removeAllShapes(); 
    pcl_v_.p->removePointCloud ("source"); //���ݸ�����ID������Ļ��ȥ��һ�����ơ�������ID
    pcl_v_.p->removePointCloud ("target"); //���ݸ�����ID������Ļ��ȥ��һ�����ơ�������ID
  }
}
