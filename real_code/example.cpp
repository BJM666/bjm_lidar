#define PCL_NO_PRECOMPILE
#define M_PI 3.1415926
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <iostream>

struct vele//define data of lidar
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

int source_flag = 0;
pcl::PointCloud<vele>::Ptr cloud_ref (new pcl::PointCloud<vele>);
ros::Publisher pose_pub;

int main (int argc, char** argv)
{
  ros::init (argc, argv, "lidar_pose");
  ros::NodeHandle nh;
  ros::Publisher pose_pub = nh.advertise<std_msgs::Float32MultiArray>("lidar_pose", 1);
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  ros::Subscriber sub = nh.subscribe ("velodyne_points", 1, cloud_cb);
  ros::spin ();
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

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PointCloud<vele>::Ptr cloud_pcl (new pcl::PointCloud<vele>);
  pcl::PointCloud<vele>::Ptr cloud_now (new pcl::PointCloud<vele>);
  pcl_conversions::toPCL(*cloud_msg, *cloud);//get to pointcloud2
  pcl::fromPCLPointCloud2 (*cloud, *cloud_pcl);//get to PointCloud1
  if (0 == source_flag) {
    cloud_ref->points.resize(cloud_pcl->points.size());
    for (size_t i=0;i<cloud_pcl->points.size();i++)
    {
      *cloud_ref = *cloud_pcl;
    }
  }
  else{
    cloud_now->points.resize(cloud_pcl->points.size());
    for (size_t i=0;i<cloud_pcl->points.size();i++)
    {
      *cloud_now = *cloud_pcl;
    }
  }
  source_flag = 1;
  pcl::IterativeClosestPoint<vele, vele> icp;
  icp.setInputSource(cloud_now);//out
  icp.setInputTarget(cloud_ref);
  icp.setMaximumIterations (100);//number of iterations
  pcl::PointCloud<vele> Final;
  icp.align(Final);
  std::vector<float> angleRPY;
  angleRPY.resize(3);
  angleRPY=matrix2angle(icp.getFinalTransformation());
  for (size_t i = 0; i < 3; i++) {
    angleRPY.push_back((float)icp.getFinalTransformation()(i,3));
  }
  /*for (size_t j = 0; j < angleRPY.size(); j++) {
    std::cout << angleRPY.at(j)<<" ";
  }*/
  std_msgs::Float32MultiArray send_msg;
  for (size_t i = 0; i < angleRPY.size(); i++) {
  send_msg.data.push_back(angleRPY.at(i));
  }
  pose_pub.publish(send_msg);
  printf("\n" );
}
