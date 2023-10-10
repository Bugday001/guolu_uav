#include <iostream>

#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>

#include <pcl/registration/gicp.h>  

#include <pcl/visualization/pcl_visualizer.h>

#include <boost/thread/thread.hpp>

#include <pcl/console/time.h>   // 利用控制台计算时间


using namespace std;


//点云旋转平移
auto TransFunc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f translation)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud, *cloud_transformed, translation);
	return cloud_transformed;
}

int main(){


    pcl::console::TicToc time;

    // -----------------加载点云数据---------------------------

    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ>("dlio_map.pcd", *target);

    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("dlio_map.pcd", *source);

    //--------------------变换旋转矩阵---------------------
	Eigen::Matrix4f translation(4, 4);
	// translation << 0.9396926, -0.3420202,  0.0000000, 5,
    //                 0.3420202,  0.9396926,  0.0000000, -3,
    //                 0.0000000,  0.0000000,  1.0000000, 0,
    //                 0, 0, 0, 1;
    translation << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
    ofstream outfile("/home/denext/catkin_ws/src/px4_offboard/params/Transformation_maxtix.yaml", ios::trunc);
	outfile << translation;
	outfile.close();
	// translation << 1, 0, 0, 0,
	// 		0, 1, 0, 0,
	// 		0, 0, 1, 0,
	// 		0, 0, 0, 1;
	auto after_trans = TransFunc(source, translation);


    cout << "读取源点云个数：" << source->points.size() << endl;

    cout << "读取目标点云个数：" << target->points.size() << endl;

    time.tic();

    //-----------------初始化GICP对象-------------------------

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp; 

    //-----------------KD树加速搜索---------------------------

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);

    tree1->setInputCloud(after_trans);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);

    tree2->setInputCloud(target);

    gicp.setSearchMethodSource(tree1);

    gicp.setSearchMethodTarget(tree2);



    //-----------------设置GICP相关参数-----------------------

    gicp.setInputSource(after_trans);  //源点云

    gicp.setInputTarget(target);  //目标点云

    gicp.setMaxCorrespondenceDistance(100); //设置对应点对之间的最大距离

    gicp.setTransformationEpsilon(1e-10);   //为终止条件设置最小转换差异

     /* gicp.setSourceCovariances(source_covariances);

    gicp.setTargetCovariances(target_covariances);*/

    gicp.setEuclideanFitnessEpsilon(0.001);  //设置收敛条件是均方误差和小于阈值， 停止迭代

    gicp.setMaximumIterations(35); //最大迭代次数  

    //gicp.setUseReciprocalCorrespondences(true);//使用相互对应关系

    // 计算需要的刚体变换以便将输入的源点云匹配到目标点云

    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    gicp.align(*icp_cloud);

    //---------------输出必要信息到显示--------------------

    cout << "Applied " << 35 << " GICP iterations in " << time.toc()/1000 << " s" << endl;

    cout << "\nGICP has converged, score is " << gicp.getFitnessScore() << endl;

    cout << "变换矩阵：\n" << gicp.getFinalTransformation() << endl;

    // 使用变换矩阵对为输入点云进行变换

    pcl::transformPointCloud(*source, *icp_cloud, gicp.getFinalTransformation());

    //pcl::io::savePCDFileASCII (".pcd", *icp_cloud);

   // -----------------点云可视化--------------------------

    boost::shared_ptr<pcl::visualization::PCLVisualizer>

        viewer_final(new pcl::visualization::PCLVisualizer("配准结果"));

    viewer_final->setBackgroundColor(0, 0, 0);  //设置背景颜色为黑色

    // 对目标点云着色可视化 (red).

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>

        target_color(target, 255, 0, 0);

    viewer_final->addPointCloud<pcl::PointXYZ>(target, target_color, "target cloud");

    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,

        1, "target cloud");

    // 对源点云着色可视化 (green).

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>

        input_color(source, 0, 255, 0);

    viewer_final->addPointCloud<pcl::PointXYZ>(source, input_color, "input cloud");

    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,

        1, "input cloud");

    // 对转换后的源点云着色 (blue)可视化.

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>

        output_color(icp_cloud, 0, 0, 255);

    viewer_final->addPointCloud<pcl::PointXYZ>(icp_cloud, output_color, "output cloud");

    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,

        1, "output cloud");

    while (!viewer_final->wasStopped())

    {

        viewer_final->spinOnce(100);

        boost::this_thread::sleep(boost::posix_time::microseconds(100000));

    }


    return (0);

}