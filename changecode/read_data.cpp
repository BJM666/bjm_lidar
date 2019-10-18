#define PCL_NO_PRECOMPILE
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

struct vele
{
  PCL_ADD_POINT4D;          // 添加pcl里xyz+padding
  float intensity;
  unsigned short ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // 确保定义新类型点云内存与SSE对齐
} EIGEN_ALIGN16;                    // 强制SSE填充以正确对齐内存
POINT_CLOUD_REGISTER_POINT_STRUCT (vele,           // 定义新类型里元素包括XYZ+“intensity”
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (unsigned short, ring, ring)
)


int
main (int argc, char** argv)//兴建一个新的点云文件，包括两个新定义点云类型的点
{
  //pcl::PointCloud<vele> cloud;//初始化点云类型
  pcl::PointCloud<vele>::Ptr cloud1 (new pcl::PointCloud<vele>);
  pcl::io::loadPCDFile<vele>("out0.pcd",*cloud1);
  std::cout<<"Loaded "
  <<cloud1->width*cloud1->height
  <<" data points from intensity_pcd.pcd with the following fields: "
  <<std::endl;
  for(size_t i=0;i<cloud1->points.size();++i)
  std::cout<<"    "<<cloud1->points[i].x
  <<" "<<cloud1->points[i].y
  <<" "<<cloud1->points[i].z
  <<" "<<cloud1->points[i].intensity
  <<" "<<cloud1->points[i].ring<<std::endl;
}
