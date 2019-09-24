

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
//#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/filter.h>
//特征
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/common/time.h>//算法时间测试

#include <Eigen/Core>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/thread/thread.hpp>

#include <pcl/features/fpfh_omp.h> //包含fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除

#include <pcl/features/vfh.h>

//pcl类型名简化
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;
typedef pcl::PointCloud<pcl::VFHSignature308> vfhFeathure;

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