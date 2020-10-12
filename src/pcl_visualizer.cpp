#include "pcl_visualizer.h"

pcl_visualizer::pcl_visualizer(bool DebugVisualizer)
{
    if(true == DebugVisualizer)
    {
        p = new pcl::visualization::PCLVisualizer ("3D Registration"); //����һ�� PCLVisualizer ����p��ȫ�ֱ���
        p->addCoordinateSystem(0.2);
        p->setCameraPosition(0.4, 0.0, -1.0, 0, -1, 0);
        p->createViewPort (0.0, 0, 0.5, 1.0, vp_1); //����������
        p->createViewPort (0.5, 0, 1.0, 1.0, vp_2); //����������
    }
}

//�ڴ��ڵ����������򵥵���ʾԴ���ƺ�Ŀ�����
void pcl_visualizer::showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud ("vp1_target"); //���ݸ�����ID������Ļ��ȥ��һ�����ơ�������ID
  p->removePointCloud ("vp1_source"); //
  pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0); //Ŀ�������ɫ
  pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0); //Դ���ƺ�ɫ
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1); //���ص���
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);
  PCL_INFO ("Press q to begin the registration.\n"); //������������ʾ��ʾ��Ϣ
  p-> spin();
}

//�ڴ��ڵ����������򵥵���ʾԴ���ƺ�Ŀ�����
void pcl_visualizer::showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
  p->removePointCloud ("source"); //���ݸ�����ID������Ļ��ȥ��һ�����ơ�������ID
  p->removePointCloud ("target");
  pcl::visualization::PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature"); //Ŀ����Ʋ�ɫ���
  if (!tgt_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");
  pcl::visualization::PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature"); //Դ���Ʋ�ɫ���
  if (!src_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");
  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2); //���ص���
  p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);
  p->spinOnce();
}

// pcl_visualizer pcl_visualizer::GetInstance()
// {
//     if(NULL == pcl_v_singleton_)
//     {
//         pcl_v_singleton_ = new pcl_visualizer();
//     }
//     return pcl_v_singleton_;
// }

