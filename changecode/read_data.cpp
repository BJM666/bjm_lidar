/*NARF (Normal Aligned Radial Feature 法向径向特征对齐）
 FPFH (Fast Point Feature histograms 快速点特征直方图)
 SIFT (Scale-invariant Feature Transform 尺寸不变特征变换)
BRIEF （Binary Robust Independent Elementary Features 二进制健壮的独立的基本特性)*/
//#include <stdafx.h>
#define PCL_NO_PRECOMPILE

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <ctime>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>  //FPFH
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>//omp加速计算
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

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
using namespace std;
typedef pcl::PointCloud<vele> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

//为了使用fpfh特征匹配，声明一个计算fpfh特征点的函数
fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud, pcl::search::KdTree<vele>::Ptr tree);

int main(int argc, char* argv[])
{

	clock_t start, end; //long
	start = clock();//开始时间
	pointcloud::Ptr source(new pointcloud);
	pointcloud::Ptr target(new pointcloud);
	pcl::io::loadPCDFile("out0.pcd", *target);
	pcl::io::loadPCDFile("outzhuan.pcd", *source);
	cout << target->size() << endl;

	//精简
	pcl::PointCloud<vele> ::Ptr target_filtered(new pcl::PointCloud<vele>);
	pcl::PointCloud<vele> ::Ptr source_filtered(new pcl::PointCloud<vele>());

	pcl::VoxelGrid<vele> sor;
	sor.setInputCloud(target);
	sor.setLeafSize(0.05, 0.05, 0.05);
	sor.filter(*target_filtered);

	pcl::VoxelGrid<vele> sor1;
	sor1.setInputCloud(source);
	sor1.setLeafSize(0.05, 0.05, 0.05);
	sor1.filter(*source_filtered);
	cout << source_filtered->size() << "  " << target_filtered->size() << endl;

	pcl::search::KdTree<vele>::Ptr tree(new pcl::search::KdTree<vele>());//创建搜索树
	fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source_filtered, tree);//计算点云点特征直方图
	fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target_filtered, tree);

	//对齐  //输入参数 ①源点云+源点特征直方图 ②目标点云+目标点特征直方图
	pcl::SampleConsensusInitialAlignment<vele, vele, pcl::FPFHSignature33> sac_ia;
	sac_ia.setInputSource(source_filtered);
	sac_ia.setSourceFeatures(source_fpfh);
	sac_ia.setInputTarget(target_filtered);
	sac_ia.setTargetFeatures(target_fpfh);
	pointcloud::Ptr final(new pointcloud);//

	//对齐参数设置
	sac_ia.setNumberOfSamples(20);
	sac_ia.setCorrespondenceRandomness(6);//设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
	//sac_ia.setMaximumIterations(100);
	sac_ia.setEuclideanFitnessEpsilon(0.001);
	sac_ia.setTransformationEpsilon(1e-10);
	sac_ia.setRANSACIterations(30);
	sac_ia.align(*final);
	cout <<"has converged:"<< sac_ia.hasConverged() <<"score"<<sac_ia.getFitnessScore()<< endl;

	end = clock();
	cout << "calculate time is " << float(end - start) / CLOCKS_PER_SEC << endl;

	//可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new
		pcl::visualization::PCLVisualizer("fpfh test"));
	int v1 = 0;
	int v2 = 1;
	view->createViewPort(0, 0, 0.5, 1, v1);
	view->createViewPort(0.5, 0, 1, 1, v2);
	view->setBackgroundColor(0, 0, 0, v1);
	view->setBackgroundColor(0.05, 0, 0, v2);

	pcl::visualization::PointCloudColorHandlerCustom<vele> source_cloud_color(source_filtered, 255, 0, 0);
	view->addPointCloud(source_filtered, source_cloud_color, "sources_cloud_v1", v1);

	pcl::visualization::PointCloudColorHandlerCustom<vele> target_cloud_color(target_filtered, 0, 255, 0);
	view->addPointCloud(target_filtered, target_cloud_color, "target_cloud_v1", v1);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sources_cloud_v1", v1);

	//
	pcl::visualization::PointCloudColorHandlerCustom<vele> aligend_cloud_color(final, 255, 0, 0);
	view->addPointCloud(final, aligend_cloud_color, "aligend_cloud_v2", v2);

	view->addPointCloud(target_filtered, target_cloud_color, "target_cloud_v2", v2);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "aligend_cloud_v2");
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud_v2");

	//对应关系可视化
	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;
	boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
	crude_cor_est.setInputSource(source_fpfh);
	crude_cor_est.setInputTarget(target_fpfh);
	//crude_cor_est.determineCorrespondences(*cru_correspondences);
	crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences);
	cout << "crude size is " << cru_correspondences->size() << endl;
	view->addCorrespondences<vele>(source, target, *cru_correspondences,"correspondence", v1);
	view->initCameraParameters();
	while (!view->wasStopped())
	{
		view->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	system("pause");
	return 0;
}

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud, pcl::search::KdTree<vele>::Ptr tree)
{
	//法向量
	pointnormal::Ptr point_normal(new pointnormal);
	pcl::NormalEstimation<vele, pcl::Normal> est_normal;
	est_normal.setInputCloud(input_cloud);
	est_normal.setSearchMethod(tree);
	est_normal.setKSearch(10);
	//est_normal.setSearchSurface();
	est_normal.compute(*point_normal);

	//fpfh估计
	fpfhFeature::Ptr fpfh(new fpfhFeature);
	//pcl::FPFHEstimation<vele, pcl::Normal, pcl::FPFHSignature33> est_target_fpfh;
	pcl::FPFHEstimationOMP<vele, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(4);//指定4核计算

	est_fpfh.setInputCloud(input_cloud);
	est_fpfh.setInputNormals(point_normal);
	est_fpfh.setSearchMethod(tree);
	est_fpfh.setKSearch(10);
	est_fpfh.compute(*fpfh);

	return fpfh;

}
