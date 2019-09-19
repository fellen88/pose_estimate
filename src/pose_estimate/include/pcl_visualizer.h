#include "MyPointRepresentation.h"

//���ӻ�
#include <pcl/visualization/pcl_visualizer.h>

#ifndef PCL_VISUALIZER_H
#define PCL_VISUALIZER_H  

class pcl_visualizer
{
    public:

    //���ӻ�����
    pcl::visualization::PCLVisualizer *p;
    //�������������������ӻ����ڷֳ�����������
    int vp_1, vp_2;

    pcl_visualizer(bool DebugVisualizer);
    void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source);
    void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source);
};
#endif