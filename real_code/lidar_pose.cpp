#define PCL_NO_PRECOMPILE
#define M_PI 3.1415926

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <math.h>

struct vele// new lidar data
{
  PCL_ADD_POINT4D;
  float intensity;
  unsigned short ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   //
} EIGEN_ALIGN16;                    //
POINT_CLOUD_REGISTER_POINT_STRUCT (vele,           //
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (unsigned short, ring, ring)
)

int
 main (int argc, char** argv)
{
    //read pcd file
  pcl::PointCloud<vele>::Ptr cloud_in (new pcl::PointCloud<vele>);
  pcl::io::loadPCDFile<vele>("out0.pcd",*cloud_in);
  pcl::PointCloud<vele>::Ptr cloud_out (new pcl::PointCloud<vele>);
  pcl::io::loadPCDFile<vele>("outzhuan.pcd",*cloud_out);

  //icp
  std::vector<float> matrix2angle(Eigen::Matrix4f rotateMatrix);//solve angle
  pcl::IterativeClosestPoint<vele, vele> icp;
  icp.setInputSource(cloud_out);//out
  icp.setInputTarget(cloud_in);//in
  //icp.setMaximumIterations (300);//number of iterations
  //icp.setMaxCorrespondenceDistance(0.5);
  //icp.setTransformationEpsilon(1e-6);
  // icp.setEuclideanFitnessEpsilon(0.11);
 // pcl::PointCloud<vele> Final;
  //icp.align(Final);

  //viewer
  std::cout << "0" << '\n';
  pcl::PointCloud<pcl::PointXYZ>::Ptr viewer_out (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr viewer_in (new pcl::PointCloud<pcl::PointXYZ>);
std::cout << "0.1" << '\n';
viewer_out->points.resize(cloud_out->points.size());
viewer_in->points.resize(cloud_in->points.size());
  for (size_t i = 0; i < cloud_out->points.size(); i++) {
      viewer_out->points[i].x=cloud_out->points[i].x;
      viewer_out->points[i].y=cloud_out->points[i].y;
      viewer_out->points[i].z=cloud_out->points[i].z;
  }
  for (size_t i = 0; i < cloud_in->points.size(); i++) {
      viewer_in->points[i].x=cloud_in->points[i].x;
      viewer_in->points[i].y=cloud_in->points[i].y;
      viewer_in->points[i].z=cloud_in->points[i].z;
  }
  int v1;
std::cout << "1" << '\n';
  pcl::visualization::PCLVisualizer viewer("icp");
  viewer.setBackgroundColor(0, 0, 0); //创建窗口
  viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v1);
  //设置点云颜色
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ref_color(viewer_in, 0, 255, 0); //投影前可以随便设一个颜色
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> present_color(viewer_out, 255, 255, 255);  //投影后的设置为白色
  viewer.addPointCloud<pcl::PointXYZ>(viewer_in, ref_color, "ref", v1);
  viewer.addPointCloud<pcl::PointXYZ>(viewer_out, present_color, "present", v1);

  std::cout << "2" << '\n';
  viewer.spin();
std::cout << "3" << '\n';

  std::vector<float> angleRPY;//get angle from mat
  angleRPY.resize(3);
  angleRPY=matrix2angle(icp.getFinalTransformation());
  std::cout  <<"x,y,z:"<<icp.getFinalTransformation()(0,3)<<" "
  <<icp.getFinalTransformation()(1,3)<<" "
  <<icp.getFinalTransformation()(2,3)<<std::endl;
  std::cout <<"R:"<< angleRPY.at(0)<<" "<<"P:"<< angleRPY.at(1)<<" "<<"Y:"<< angleRPY.at(2)<<" "<<std::endl;
  printf("\n" );
  std::cout <<icp.getFinalTransformation()<<std::endl;
 return (0);
}

std::vector<float> matrix2angle(Eigen::Matrix4f rotateMatrix)//solve angle
{
	float sy = (float)sqrt(rotateMatrix(0,0) * rotateMatrix(0,0) + rotateMatrix(1,0)*rotateMatrix(1,0));
	bool singular = sy < 1e-6; // If
	float x, y, z;
	if (!singular)
	{
		x = (float)atan2(rotateMatrix(2,1), rotateMatrix(2,2));
		y = (float)atan2(-rotateMatrix(2,0), sy);
		z = (float)atan2(rotateMatrix(1, 0), rotateMatrix(0, 0));
	}
	else
	{
		x = (float)atan2(-rotateMatrix(1, 2), rotateMatrix(1, 1));
		y = (float)atan2(-rotateMatrix(2, 0), sy);
		z = 0;
	}
	std::vector<float> angle;
	angle.push_back((float)(x * (180.0f / M_PI)));
	angle.push_back((float)(y * (180.0f / M_PI)));
	angle.push_back((float)(z * (180.0f / M_PI)));
	return angle;
}
