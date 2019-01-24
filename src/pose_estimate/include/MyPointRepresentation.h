

#ifndef MYPOINTREPRESENTATION_H
#define MYPOINTREPRESENTATION_H

//��/����
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

//pcd�ļ�����/���
#include <pcl/io/pcd_io.h>
//�˲�
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
//����
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>

//pcl��������
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// 定义新的点表达方�?< x, y, z, curvature > 坐标+曲率
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT> //继承关系
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
  public:
  MyPointRepresentation ()
  {
    //指定维数
    nr_dimensions_ = 4;
  }
  //重载函数copyToFloatArray，以定义�?己的特征向量
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    //< x, y, z, curvature > 坐标xyz和曲�?
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};
# endif