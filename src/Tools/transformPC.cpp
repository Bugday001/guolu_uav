// 包含必要的头文件
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <vector>
using namespace std;
// 定义点云类型
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// 主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "transformPC");
    std::cout<<"start!!"<<std::endl;
    ros::NodeHandle nh;
    vector<float> extRotV;
    string input_path, output_path;
    nh.param<vector<float>>("/transformation_node/transformation", extRotV, vector<float>());
    nh.param<string>("/transformation_node/input_path", input_path, "");
    nh.param<string>("/transformation_node/output_path", output_path, "");
    Eigen::Matrix4f transform = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(extRotV.data(), 4, 4);
    // // 读取点云数据文件
    PointCloud::Ptr cloud(new PointCloud);
    if (pcl::io::loadPLYFile(input_path, *cloud) != 0)
    {
        PCL_ERROR ("Error loading ply file\n");
        return (-1);
    }
    // // 定义变换矩阵，这里使用了一个沿X轴平移2米，绕Z轴旋转45度的变换
    // // Eigen::Matrix4f transform;
    // // transform << 0.453207, -0.891405, 0, 3.12513,
    // //             0.889788, 0.452385, 0.0602092, -7.09262,
    // //             -0.0536708, -0.0272873, 0.998186,
    // //             0, 0, 0, 0, 1;

    // // 打印变换矩阵
    std::cout << "Transform matrix:" << std::endl;
    std::cout << transform << std::endl;

    // 创建一个新的点云，用于存储变换后的结果
    PointCloud::Ptr transformed_cloud(new PointCloud);

    // 使用pcl::transformPointCloud函数进行变换
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    pcl::PLYWriter writer;
    writer.write(output_path, *transformed_cloud, true);

    // // 创建一个可视化对象，并添加原始点云和变换后的点云
    // pcl::visualization::PCLVisualizer viewer("Point Cloud Transform");
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud, 255, 255, 255);                // 白色
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_color(transformed_cloud, 230, 20, 20); // 红色
    // viewer.addPointCloud(cloud, source_color, "source");
    // viewer.addPointCloud(transformed_cloud, transformed_color, "transformed");

    // // 设置背景颜色和渲染属性
    // viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // 设置背景颜色为深灰色
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed");

    

    // // 启动可视化循环
    // while (!viewer.wasStopped())
    // {
    //     viewer.spinOnce();
    //     pcl_sleep(0.01);
    //     if (viewer.wasStopped())
    //     {
    //         break;
    //     }
    // }

    // ros::spin();
    return (0);
}
