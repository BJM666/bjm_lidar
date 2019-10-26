#define PCL_NO_PRECOMPILE
#define M_PI 3.1415926

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
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
  icp.setMaximumIterations (300);//number of iterations
  //icp.setMaxCorrespondenceDistance(0.5);
  //icp.setTransformationEpsilon(1e-6);
  // icp.setEuclideanFitnessEpsilon(0.11);
  pcl::PointCloud<vele> Final;
  icp.align(Final);

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
