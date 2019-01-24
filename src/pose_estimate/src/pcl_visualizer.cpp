#include "pcl_visualizer.h"

pcl_visualizer::pcl_visualizer(bool DebugVisualizer)
{
    if(true == DebugVisualizer)
    {
        p = new pcl::visualization::PCLVisualizer ("3D Registration"); //创建一个 PCLVisualizer 对象，p是全局变量
        p->addCoordinateSystem(0.2);
        p->setCameraPosition(0.4, 0.0, -1.0, 0, -1, 0);
        p->createViewPort (0.0, 0, 0.5, 1.0, vp_1); //创建左视区
        p->createViewPort (0.5, 0, 1.0, 1.0, vp_2); //创建右视区
    }
}

//在窗口的左视区，简单的显示源点云和目标点云
void pcl_visualizer::showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud ("vp1_target"); //根据给定的ID，从屏幕中去除一个点云。参数是ID
  p->removePointCloud ("vp1_source"); //
  pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0); //目标点云绿色
  pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0); //源点云红色
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1); //加载点云
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);
  PCL_INFO ("Press q to begin the registration.\n"); //在命令行中显示提示信息
  p-> spin();
}

//在窗口的右视区，简单的显示源点云和目标点云
void pcl_visualizer::showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
  p->removePointCloud ("source"); //根据给定的ID，从屏幕中去除一个点云。参数是ID
  p->removePointCloud ("target");
  pcl::visualization::PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature"); //目标点云彩色句柄
  if (!tgt_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");
  pcl::visualization::PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature"); //源点云彩色句柄
  if (!src_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");
  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2); //加载点云
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

