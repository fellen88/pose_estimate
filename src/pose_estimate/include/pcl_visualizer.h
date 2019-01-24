#include "MyPointRepresentation.h"

//可视化
#include <pcl/visualization/pcl_visualizer.h>

#ifndef PCL_VISUALIZER_H
#define PCL_VISUALIZER_H  

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