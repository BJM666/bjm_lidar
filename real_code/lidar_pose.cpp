#define PCL_NO_PRECOMPILE

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <math.h>

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
 main (int argc, char** argv)
{
  pcl::PointCloud<vele>::Ptr cloud_in1 (new pcl::PointCloud<vele>);
  pcl::io::loadPCDFile<vele>("in1.pcd",*cloud_in1);
  //pcl::PointCloud<vele>::Ptr cloud_in2 (new pcl::PointCloud<vele>);
//  pcl::io::loadPCDFile<vele>("in2.pcd",*cloud_in2);


  pcl::PointCloud<vele>::Ptr cloud_out1 (new pcl::PointCloud<vele>);
  pcl::io::loadPCDFile<vele>("qian1.pcd",*cloud_out1);
//  pcl::PointCloud<vele>::Ptr cloud_out2 (new pcl::PointCloud<vele>);
  //pcl::io::loadPCDFile<vele>("out2.pcd",*cloud_out2);


  pcl::PointCloud<vele>::Ptr cloud_in (new pcl::PointCloud<vele>);
  pcl::PointCloud<vele>::Ptr cloud_out (new pcl::PointCloud<vele>);

  std::vector< pcl::PointCloud<vele> > combine_in_point;
  combine_in_point.push_back(*cloud_in1);
  //combine_in_point.push_back(*cloud_in2);
  for (size_t i=0;i<combine_in_point.size();i++)
  {
  	*cloud_in += combine_in_point[i];
  }

  std::vector< pcl::PointCloud<vele> > combine_out_point;
  combine_out_point.push_back(*cloud_out1);
  //combine_out_point.push_back(*cloud_out2);
  for (size_t i=0;i<combine_out_point.size();i++)
  {
  	*cloud_out += combine_out_point[i];
  }

  pcl::IterativeClosestPoint<vele, vele> icp;
  icp.setInputSource(cloud_in);//out
  icp.setInputTarget(cloud_out);
  icp.setMaximumIterations (100);//number of iterations
  pcl::PointCloud<vele> Final;
  icp.align(Final);
  //std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  //icp.getFitnessScore() << std::endl;
  /*float T[4][4]=icp.getFinalTransformation();
  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < j; j++) {
      std::cout << T[i][j] << std::endl;
    }
    printf("\n" );
  }*/
  std::cout << icp.getFinalTransformation()<< std::endl;
  //float r,p,y;//roll,pitch,yaw
  //p=asin(-icp.getFinalTransformation()(0,2));
  //y=atan(icp.getFinalTransformation()(0,1)/icp.getFinalTransformation()(0,2));
  //r=atan(icp.getFinalTransformation()(1,2)/icp.getFinalTransformation()(2,2));
  //std::cout << r<<p<<y<< std::endl;
 return (0);
}
