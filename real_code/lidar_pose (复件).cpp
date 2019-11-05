#define PCL_NO_PRECOMPILE
#define M_PI 3.1415926

#include <iostream>
//#include <omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/keypoints/sift_keypoint.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/octree/octree.h>
#include <time.h>
#include <math.h>
#include <pcl/registration/ndt.h>

//#include <vector>
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
/*namespace pcl
{
  template<>
    struct SIFTKeypointFieldSelector<vele>
    {
      inline float
      operator () (const vele &p) const
      {
    return p.z;
      }
    };
}*/


int main (int argc, char** argv)
{
    //recordtime
    clock_t start,finish,predeal,keypoints;
    double totaltime,time_key,time_icp,time_pre;
    //read pcd file
  pcl::PointCloud<vele>::Ptr cloud_in (new pcl::PointCloud<vele>);
  pcl::io::loadPCDFile<vele>("out0.pcd",*cloud_in);
  pcl::PointCloud<vele>::Ptr cloud_out (new pcl::PointCloud<vele>);
  pcl::io::loadPCDFile<vele>("outzhuan.pcd",*cloud_out);
  start=clock();
  //predeal
  pcl::PointCloud<vele>::Ptr cloud_in_filtering (new pcl::PointCloud<vele>);
  pcl::PointCloud<vele>::Ptr cloud_out_filtering (new pcl::PointCloud<vele>);
  for (size_t i = 0; i < cloud_in->points.size(); i++) {
      if (cloud_in->points[i].intensity > 70) {
        cloud_in_filtering->push_back(cloud_in->points[i]);
         // cloud_in->erase(cloud_in->begin()+i);
      }
  }
  for (size_t i = 0; i < cloud_out->points.size(); i++) {
      if (cloud_out->points[i].intensity > 70) {
        cloud_out_filtering->push_back(cloud_out->points[i]);
         // cloud_in->erase(cloud_in->begin()+i);
      }
  }
  pcl::PointCloud<vele>::Ptr cloud_in_filtering1 (new pcl::PointCloud<vele>);
  pcl::PointCloud<vele>::Ptr cloud_out_filtering1 (new pcl::PointCloud<vele>);

    pcl::VoxelGrid<vele> voxel_filter;
    voxel_filter.setLeafSize (0.1f, 0.1f, 0.1f);
    voxel_filter.setInputCloud (cloud_in_filtering);
    voxel_filter.filter (*cloud_in_filtering1);
    voxel_filter.setInputCloud (cloud_out_filtering);
    voxel_filter.filter (*cloud_out_filtering1);

    predeal=clock();
    //ndt
    //pcl::PointCloud<vele>::Ptr cloud_in_filtered (new pcl::PointCloud<vele>);
    /*pcl::PointCloud<vele>::Ptr cloud_out_filtered (new pcl::PointCloud<vele>);
    pcl::NormalDistributionsTransform<vele, vele> ndt;
    ndt.setMaximumIterations(50);
    //ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.1);
    ndt.setResolution(0.1);
    ndt.setInputTarget(cloud_in);
    ndt.setInputSource(cloud_out_filtering1);
    ndt.align(*cloud_out_filtered);*/
    //pcl::search::KdTree<vele>::Ptr tree_out(new pcl::search::KdTree<vele>());
    //pcl::search::KdTree<vele>::Ptr tree_in(new pcl::search::KdTree<vele>());
    //tree_in->setInputCloud(cloud_in_filtering1);
    //tree_out->setInputCloud(cloud_out_filtering1);
    //pcl::PointCloud<vele>::Ptr cloud_in_filtered (new pcl::PointCloud<vele>);
    //pcl::PointCloud<vele>::Ptr cloud_out_filtered (new pcl::PointCloud<vele>);
    /*const float min_scale = 0.05f;
    const int n_octaves = 10;
    const int n_scales_per_octave = 4;
    const float min_contrast = 0.05f;
    pcl::SIFTKeypoint<vele, pcl::PointWithScale> sift_in;//����sift�ؼ�����������

    pcl::PointCloud<pcl::PointWithScale> result_in;
    sift_in.setInputCloud(cloud_in_filtering1);//������������
    sift_in.setSearchMethod(tree_in);//����һ���յ�kd������tree�����������ݸ�sift��������
    sift_in.setScales(min_scale, n_octaves, n_scales_per_octave);//ָ�������ؼ����ĳ߶ȷ�Χ
    sift_in.setMinimumContrast(min_contrast);//�������ƹؼ�����������ֵ
    sift_in.compute(result_in);//ִ��sift�ؼ������⣬����������result
    copyPointCloud(result_in, *cloud_in_filtered);

    pcl::SIFTKeypoint<vele, pcl::PointWithScale> sift_out;//����sift�ؼ�����������
    pcl::PointCloud<pcl::PointWithScale> result_out;
    sift_out.setInputCloud(cloud_out_filtering1);//������������
    sift_out.setSearchMethod(tree_out);//����һ���յ�kd������tree�����������ݸ�sift��������
    sift_out.setScales(min_scale, n_octaves, n_scales_per_octave);//ָ�������ؼ����ĳ߶ȷ�Χ
    sift_out.setMinimumContrast(min_contrast);//�������ƹؼ�����������ֵ
    sift_out.compute(result_out);//ִ��sift�ؼ������⣬����������result
    copyPointCloud(result_out, *cloud_out_filtered);*/
    keypoints=clock();
  //icp
    std::vector<float> matrix2angle(Eigen::Matrix4f rotateMatrix);//solve angle
    pcl::IterativeClosestPoint<vele,vele> icp;
  icp.setInputSource(cloud_out_filtering);//out
  icp.setInputTarget(cloud_in_filtering);//in
  //icp.setSearchMethodSource(tree_out);
  //icp.setSearchMethodTarget(tree_in);
  icp.setMaximumIterations (40);//number of iterations
  //icp.setMaxCorrespondenceDistance(3);
  //icp.setTransformationEpsilon(1e-6);
  //icp.setEuclideanFitnessEpsilon(0.11);
  pcl::PointCloud<vele>::Ptr Final (new pcl::PointCloud<vele>);
  //pcl::PointCloud<vele> Final;
  icp.align(*Final);
  std::vector<float> angleRPY;//get angle from mat
  angleRPY.resize(3);
  angleRPY=matrix2angle(icp.getFinalTransformation());
  std::cout  <<"x,y,z:"<<icp.getFinalTransformation()(0,3)<<" "
  <<icp.getFinalTransformation()(1,3)<<" "
  <<icp.getFinalTransformation()(2,3)<<std::endl;
  std::cout <<"R:"<< angleRPY.at(0)<<" "<<"P:"<< angleRPY.at(1)<<" "<<"Y:"
  << angleRPY.at(2)<<" "<<std::endl;
  printf("\n" );
  std::cout <<icp.getFinalTransformation()<<std::endl;
    finish=clock();
    time_pre=(double)(predeal-start)/CLOCKS_PER_SEC;
    time_key=(double)(keypoints-predeal)/CLOCKS_PER_SEC;
    time_icp=(double)(finish-keypoints)/CLOCKS_PER_SEC;
    totaltime=(double)(finish-start)/CLOCKS_PER_SEC;

    cout<<"\n滤波时间为"<<time_pre<<"秒！"<<endl;
    cout<<"\n查找关键点时间为"<<time_key<<"秒！"<<endl;
    cout<<"\nicp时间为"<<time_icp<<"秒！"<<endl;
    cout<<"\n总时间为"<<totaltime<<"秒！"<<endl;
    cout<<"\n初始点数为"<<cloud_in->points.size()<<"秒！"<<endl;
    //cout<<"\n下采样后点数为"<<cloud_in_filtering->points.size()<<"秒！"<<endl;
    cout<<"\n关键点数为"<< cloud_in_filtering1->points.size() <<"秒！"<<endl;

  //viewer
  int v1;
  pcl::visualization::PCLVisualizer viewer("icp");
  viewer.setBackgroundColor(0, 0, 0); //创建窗口
  viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v1);
  pcl::visualization::PointCloudColorHandlerCustom<vele> target_color(cloud_in_filtering1, 255, 255, 255); //目标白
  //pcl::visualization::PointCloudColorHandlerCustom<vele> ndt_color(cloud_out_filtered, 0, 0, 255);  //ndt蓝
  pcl::visualization::PointCloudColorHandlerCustom<vele> icp_color(Final, 0, 255, 0);  //ICP绿
  pcl::visualization::PointCloudColorHandlerCustom<vele> source_color(cloud_out_filtering1, 255, 0, 0);  //初始红
  viewer.addPointCloud<vele>(cloud_in_filtering1, target_color, "target", v1);
  //viewer.addPointCloud<vele>(cloud_out_filtered, ndt_color, "ndt", v1);
  viewer.addPointCloud<vele>(Final, icp_color, "icp", v1);
  viewer.addPointCloud<vele>(cloud_out_filtering1, source_color, "source", v1);
  viewer.spin();

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
