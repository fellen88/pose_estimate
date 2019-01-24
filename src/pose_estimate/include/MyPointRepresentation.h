

#ifndef MYPOINTREPRESENTATION_H
#define MYPOINTREPRESENTATION_H

//点/点云
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

//pcd文件输入/输出
#include <pcl/io/pcd_io.h>
//滤波
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
//特征
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>

//pcl类型名简化
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// 瀹氫箟鏂扮殑鐐硅〃杈炬柟寮?< x, y, z, curvature > 鍧愭爣+鏇茬巼
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT> //缁ф壙鍏崇郴
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
  public:
  MyPointRepresentation ()
  {
    //鎸囧畾缁存暟
    nr_dimensions_ = 4;
  }
  //閲嶈浇鍑芥暟copyToFloatArray锛屼互瀹氫箟鑷?宸辩殑鐗瑰緛鍚戦噺
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    //< x, y, z, curvature > 鍧愭爣xyz鍜屾洸鐜?
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};
# endif