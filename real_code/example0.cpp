#define PCL_NO_PRECOMPILE
#define M_PI 3.1415926
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float32MultiArray.h>
#include <time.h>
#include <math.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <time.h>

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

std::vector<pcl::PointCloud<vele>::Ptr > v;
clock_t start,finish,predeal,keypoints;
class velepointcloud
{
public:
    velepointcloud()
    {
         min_scale = 0.1f;
         n_octaves = 6;
         n_scales_per_octave = 2;
         min_contrast = 0.1f;
         LeafSize=0.1f;
    }
    pcl::PointCloud<vele>::Ptr voxel_filter(pcl::PointCloud<vele>::Ptr cloud_in)
    {
        pcl::PointCloud<vele>::Ptr cloud_out (new pcl::PointCloud<vele>);
        pcl::VoxelGrid<vele> vox_filter;
        vox_filter.setLeafSize (LeafSize,LeafSize,LeafSize);
        vox_filter.setInputCloud (cloud_in);
        vox_filter.filter (*cloud_out);
        return cloud_out;
    }
    pcl::PointCloud<vele>::Ptr findkeypoint(pcl::PointCloud<vele>::Ptr cloud_in)
    {
        pcl::PointCloud<vele>::Ptr cloud_out (new pcl::PointCloud<vele>);
        pcl::search::KdTree<vele>::Ptr tree_in(new pcl::search::KdTree<vele>());
        pcl::SIFTKeypoint<vele, pcl::PointWithScale> sift_in;
        pcl::PointCloud<pcl::PointWithScale> result_in;
        sift_in.setInputCloud(cloud_in);
        sift_in.setSearchMethod(tree_in);
        sift_in.setScales(min_scale, n_octaves, n_scales_per_octave);
        sift_in.setMinimumContrast(min_contrast);
        sift_in.compute(result_in);
        copyPointCloud(result_in, *cloud_out);
        return cloud_out;
    }

private:
    float min_scale;
    int n_octaves;
    int n_scales_per_octave;
    float min_contrast;
    float LeafSize;
};
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {    //Topic you want to publish
    pub_ = n_.advertise<std_msgs::Float32MultiArray>("lidarpose", 1);
    sub_ = n_.subscribe("velodyne_points", 1, &SubscribeAndPublish::callback, this);
  }
  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
};
int main(int argc, char **argv)
{
    double totaltime,time_key,time_icp,time_pre;
  ros::init(argc, argv, "subscribe_and_publish");
  std::vector<float> matrix2angle(Eigen::Matrix4f rotateMatrix);
  SubscribeAndPublish SAPObject;
  ros::spin();
  return 0;
}
void SubscribeAndPublish::callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    start=clock();

    std::vector<float> matrix2angle(Eigen::Matrix4f rotateMatrix);//solve angle
  pcl::PCLPointCloud2* cloud2_0 = new pcl::PCLPointCloud2;
  pcl::PointCloud<vele>::Ptr cloud_0 (new pcl::PointCloud<vele>);
  pcl_conversions::toPCL(*cloud_msg, *cloud2_0);//get to pointcloud2
  pcl::fromPCLPointCloud2 (*cloud2_0, *cloud_0);//get to PointCloud1
  //pcl::PointCloud<vele>::Ptr cloud_0 (new pcl::PointCloud<vele>);
  predeal=clock();
  //pcl::PointCloud<vele>::Ptr cloud_1 (new pcl::PointCloud<vele>);
  /*if (0 == source_flag)
  {
    cloud_0->points.resize(cloud_pcl->points.size());
    for (size_t i=0;i<cloud_pcl->points.size();i++)
    {
      *cloud_0 = *cloud_pcl;
      source_flag =1;
    }
  }
    cloud_1->points.resize(cloud_pcl->points.size());
    for (size_t i=0;i<cloud_pcl->points.size();i++)
    {
      *cloud_1 = *cloud_pcl;
      source_flag = 0;
  }*/

  pcl::PointCloud<vele>::Ptr cloud_0_filter (new pcl::PointCloud<vele>);
  //pcl::PointCloud<vele>::Ptr cloud_1_filter (new pcl::PointCloud<vele>);
  pcl::PointCloud<vele>::Ptr cloud_0_keypoint (new pcl::PointCloud<vele>);
  //pcl::PointCloud<vele>::Ptr cloud_1_keypoint (new pcl::PointCloud<vele>);
  velepointcloud cloud0;
  cloud_0_filter=cloud0.voxel_filter(cloud_0);
  keypoints=clock();
  cloud_0_keypoint=cloud0.findkeypoint(cloud_0_filter);
  //cloud_1_filter=cloud0.voxel_filter(cloud_1);
  //cloud_1_keypoint=cloud0.findkeypoint(cloud_1_filter);

  v.push_back(cloud_0_keypoint);
  if (v.size()>1) {
    pcl::IterativeClosestPoint<vele, vele> icp;
    icp.setInputSource(v[1]);//out
    icp.setInputTarget(v[0]);
    icp.setMaximumIterations (20);//number of iterations
    pcl::PointCloud<vele> Final;
    icp.align(Final);
    std::vector<pcl::PointCloud<vele>::Ptr>::iterator begin = v.begin();
    v.erase(begin);
    std::vector<float> angleRPY;
    angleRPY.resize(3);
    angleRPY=matrix2angle(icp.getFinalTransformation());
    for (size_t i = 0; i < 3; i++) {
        angleRPY.push_back((float)icp.getFinalTransformation()(i,3));
    }
    //outangle
    std_msgs::Float32MultiArray send_msg;
    for (size_t i = 0; i < angleRPY.size(); i++) {
        send_msg.data.push_back(angleRPY.at(i));
    }
    pub_.publish(send_msg);
  }
  //std_msgs::Float32MultiArray output;
  finish=clock();
  std::cout << "总时间" << (double)(finish-start)/CLOCKS_PER_SEC<<'\n';
  std::cout << "滤波" << (double)(predeal-start)/CLOCKS_PER_SEC<<'\n';
  std::cout << "关键点" << (double)(keypoints-predeal)/CLOCKS_PER_SEC<<'\n';
  std::cout << "迭代" << (double)(finish-keypoints)/CLOCKS_PER_SEC<<'\n';
  std::cout << "下采样点数" <<cloud_0_filter->points.size() <<'\n';
  std::cout << "关键点数" <<cloud_0_keypoint->points.size() <<'\n';
  std::cout << "" << '\n';
  //std::cout << "位姿" << send_msg.data.at(0)<<'\n';
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
