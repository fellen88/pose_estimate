//���ӻ�
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h> //ֱ��ͼ�Ŀ��ӻ�
#include <pcl/visualization/pcl_plotter.h>// ֱ��ͼ�Ŀ��ӻ� ����2

#ifndef PCL_VISUALIZER_H
#define PCL_VISUALIZER_H  

//pcl��������
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

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