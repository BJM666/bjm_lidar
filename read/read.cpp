#define PCL_NO_PRECOMPILE
#define M_PI 3.1415926
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

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
int main(){
    pcl::PointCloud<vele>::Ptr cloud_in (new pcl::PointCloud<vele>);
    pcl::PointCloud<vele>::Ptr cloud_out (new pcl::PointCloud<vele>);

    pcl::io::loadPCDFile<vele>("out0.pcd",*cloud_in);

    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    const float speed = 2.5;
    const float angle = M_PI;
    const float frequency =5;
    float theta = M_PI/frequency; // The angle of rotation in radians
    float distance =speed/frequency;
    transform_1 (0,0) = cos (theta);
    transform_1 (0,1) = -sin(theta);
    transform_1 (1,0) = sin (theta);
    transform_1 (1,1) = cos (theta);
    transform_1 (0,3) = distance;
    transform_1 (1,3) = distance;
    transform_1 (2,3) = distance;
    pcl::transformPointCloud (*cloud_in, *cloud_out, transform_1);

    int v1;
    pcl::visualization::PCLVisualizer viewer("viewer");
    viewer.setBackgroundColor(0, 0, 0); //创建窗口
    viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v1);
    pcl::visualization::PointCloudColorHandlerCustom<vele> in_color(cloud_in,0, 255, 255); //目标白
    pcl::visualization::PointCloudColorHandlerCustom<vele> out_color(cloud_out, 0, 255, 255); //目标白
    viewer.addPointCloud<vele>(cloud_in, in_color, "in", v1);
    viewer.addPointCloud<vele>(cloud_out, out_color, "out", v1);
    pcl::io::savePCDFileASCII("outout.pcd",*cloud_out);
    std::cout << transform_1 << '\n';
    viewer.spin();
    return 0;
}
