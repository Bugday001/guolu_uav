/**
 * 将三维激光雷达点云与已知pcd进行匹配，得到初始位置，用于任务点的发布
 */
#include <boost/smart_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <pcl/registration/gicp.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>

using namespace pcl;
using namespace std;
std::string pcd_target_path;
Eigen::Matrix4f guess;
/**
 *    可视化
*/
void viewer(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source, 
pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_result, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_target_cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
        viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color0(cloud_result, 0, 0, 255); // 蓝
    viewer_final->addPointCloud<pcl::PointXYZ>(cloud_source, color0, "cloud0");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud0");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud_result, 255, 0, 0); // 红
    viewer_final->addPointCloud<pcl::PointXYZ>(cloud_result, color1, "cloud1");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(filtered_target_cloud, 0, 255, 0); // 绿
    viewer_final->addPointCloud<pcl::PointXYZ>(filtered_target_cloud, color2, "cloud2");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2");

    viewer_final->addCoordinateSystem(1.0);
    viewer_final->initCameraParameters();

    while (!viewer_final->wasStopped())
    {
        viewer_final->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

/**
 * PointToPlane-ICP
*/
Eigen::Matrix4f PointToPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr src(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*cloud1, *src);
    pcl::PointCloud<pcl::PointNormal>::Ptr tgt(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*cloud2, *tgt);

    // 估算法向量
    pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est;
    norm_est.setSearchMethod(pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>));
    norm_est.setKSearch(10);
    norm_est.setInputCloud(tgt);
    norm_est.compute(*tgt);

    pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
    typedef pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal> PointToPlane;
    boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
    icp.setTransformationEstimation(point_to_plane); // key

    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    icp.setRANSACIterations(50);
    icp.setMaximumIterations(200);
    icp.setTransformationEpsilon(1e-3);
    pcl::PointCloud<pcl::PointNormal> output;
    icp.align(output, guess);
    return icp.getFinalTransformation();
}

void voxel_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &inut_cloud,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud, double leaf_size)
{

    pcl::VoxelGrid<pcl::PointXYZ> vf;
    vf.setLeafSize(leaf_size, leaf_size, leaf_size);
    vf.setInputCloud(inut_cloud);
    vf.filter(*filtered_cloud);

    std::vector<int> indices; // 保存去除的点的索引
    pcl::removeNaNFromPointCloud(*filtered_cloud, *filtered_cloud, indices);
}

void doubleICP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudicp(new pcl::PointCloud<pcl::PointXYZ>);
    //icp的收敛参数
    icp.setTransformationEpsilon(1e-10);  // 均方误差
    icp.setEuclideanFitnessEpsilon(0.01); // 这个是论文部分对应的三次连续变换的差值部分的收敛情况
    icp.setMaximumIterations(200);        // 最大迭代次数等
    icp.setInputCloud(cloud1);
    icp.setInputTarget(cloud2);
    icp.align(*cloudicp); 
    cout << "point to point icp finished" << endl;
    Eigen::Matrix4f trans = PointToPlane(cloudicp, cloud2);
    Eigen::Matrix4f total_trans = icp.getFinalTransformation() * trans;
    std::cout << "double ICP 配准后的变换矩阵: \n" << total_trans <<std::endl;
    Eigen::Matrix3f I= Eigen::MatrixXf::Identity(3,3);
    std::cout << "变换坐标系后的平移向量t: \n" << total_trans.block(0,0,3,3).ldlt().solve(I)*total_trans.block(0,3,3,1) <<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloudicp, *cloud_result, trans);
    viewer(cloud1, cloud_result, cloud2);
}

Eigen::Matrix4f directICP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
{
    Eigen::Matrix4f trans = PointToPlane(cloud1, cloud2);
    std::cout << "Point2Plane配准后的变换矩阵: \n" << trans <<std::endl;
    Eigen::Matrix3f I= Eigen::MatrixXf::Identity(3,3);
    std::cout << "变换坐标系后的平移向量t: \n" << trans.block(0,0,3,3).ldlt().solve(I)*trans.block(0,3,3,1) <<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f test_trans = Eigen::MatrixXf::Identity(4, 4);  //只包含平移变换部分t
    test_trans.col(3) = trans.col(3);
    pcl::transformPointCloud(*cloud1, *cloud_result, trans);
    viewer(cloud1, cloud_result, cloud2);
    return trans;
}

/**
 * 将解算得到的变换矩阵写入yaml
*/
void test_save_txt(Eigen::Matrix4f mat, string filename)
{
	ofstream outfile(filename, ios::trunc);
	outfile << mat;
	outfile.close();
}

int flag = 0;
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud)
{
    if(flag) return;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>); // 目标文件
    pcl::fromROSMsg(*cloud, *pc);
    flag = 1;
    // std::string pcd_target_path = ros::package::getPath("px4_offboard")+"/models/dlio_map.pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>); // 目标文件
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(pcd_target_path, *cloud_target) == -1)
    {
        PCL_ERROR("Couldn't read file cloud2\n");
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter(cloud_target, filtered_target_cloud, 0.01);
    Eigen::Matrix4f trans = directICP(pc, filtered_target_cloud);
    test_save_txt(trans, ros::package::getPath("px4_offboard")+"/params/Transformation_maxtix.yaml");
}

// 主函数入口
int main(int argc, char **argv)
{
    ros::init(argc, argv, "matchHome");
    ros::NodeHandle nh;
    nh.param<string>("/matchHome_node/output_path", pcd_target_path, "");
    std::string pointCloud_tag = "/robot/dlio/odom_node/pointcloud/deskewed";
    vector<float> extRotV;
    nh.param<vector<float>>("/matchHome_node/transformation", extRotV, vector<float>());
    guess = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(extRotV.data(), 4, 4);
    ros::Subscriber subscriber = nh.subscribe(pointCloud_tag, 10, &pointCloudCallback);
    ros::spin();
    return 0;
}

