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
  pcl::PointCloud<vele>::Ptr cloud_in (new pcl::PointCloud<vele>);
  pcl::io::loadPCDFile<vele>("ref.pcd",*cloud_in);
  pcl::PointCloud<vele>::Ptr cloud_out (new pcl::PointCloud<vele>);
  pcl::io::loadPCDFile<vele>("present.pcd",*cloud_out);
  std::vector<float> matrix2angle(Eigen::Matrix4f rotateMatrix);//solve angle

//  pcl::PointCloud<vele>::Ptr cloud_in (new pcl::PointCloud<vele>);
//  pcl::PointCloud<vele>::Ptr cloud_out (new pcl::PointCloud<vele>);

  //std::vector< pcl::PointCloud<vele> > combine_in_point;
  //combine_in_point.push_back(*cloud_in1);
  //combine_in_point.push_back(*cloud_in2);
  //for (size_t i=0;i<combine_in_point.size();i++)
  //{
  //	*cloud_in += combine_in_point[i];
//  }

//  std::vector< pcl::PointCloud<vele> > combine_out_point;
//  combine_out_point.push_back(*cloud_out1);
  //combine_out_point.push_back(*cloud_out2);
//  for (size_t i=0;i<combine_out_point.size();i++)
//  {
//  	*cloud_out += combine_out_point[i];
//  }

  pcl::IterativeClosestPoint<vele, vele> icp;
  icp.setInputSource(cloud_out);//out
  icp.setInputTarget(cloud_in);
  icp.setMaximumIterations (300);//number of iterations
  pcl::PointCloud<vele> Final;
  icp.align(Final);
  std::vector<float> angleRPY;
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
