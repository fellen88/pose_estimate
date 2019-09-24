//可视化
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h> //直方图的可视化
#include <pcl/visualization/pcl_plotter.h>// 直方图的可视化 方法2

#ifndef PCL_VISUALIZER_H
#define PCL_VISUALIZER_H  

//pcl类型名简化
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

class pcl_visualizer
{
    public:

    //可视化对象
    pcl::visualization::PCLVisualizer *p;
    //左视区和右视区，可视化窗口分成左右两部分
    int vp_1, vp_2;

    pcl_visualizer(bool DebugVisualizer);
    void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source);
    void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source);
};
#endif