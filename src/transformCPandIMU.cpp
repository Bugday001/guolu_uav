#include <iostream>
#include <tf/tf.h>
// ros相关节点
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

geometry_msgs::PoseStamped visionPose;
geometry_msgs::PoseStamped visionPosition;
geometry_msgs::Twist visionVelocity;
ros::Publisher cloud_pub, velPub, visionPosPub;
ros::Timer timer;

struct Point
{
    Point() : data{0.f, 0.f, 0.f, 1.f} {}

    PCL_ADD_POINT4D;
    float intensity; // intensity
    union
    {
        std::uint32_t t;           // OUSTER: time since beginning of scan in nanoseconds
        float time;                // VELODYNE: time since beginning of scan in seconds
        std::uint32_t offset_time; // LIVOX: time from beginning of scan in nanoseconds
        double timestamp;          // HESAI: absolute timestamp in seconds
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint32_t, t, t)(float, time, time)(std::uint32_t, offset_time, offset_time)(double, timestamp, timestamp))
typedef Point PointType;

Eigen::Matrix4f rotMatrix=Eigen::Matrix4f::Zero();;


void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<PointType>::Ptr original_scan_(boost::make_shared<pcl::PointCloud<PointType>>());
    pcl::fromROSMsg(*msg, *original_scan_);
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    pcl::transformPointCloud(*original_scan_, *cloud, rotMatrix);
    sensor_msgs::PointCloud2 cloud_ros;
    pcl::toROSMsg(*cloud, cloud_ros);
    cloud_ros.header.stamp = msg->header.stamp;
    cloud_ros.header.seq = msg->header.seq;
    cloud_ros.header.frame_id = "base_link";
    cloud_pub.publish(cloud_ros);
}

int main(int argc, char **argv)
{

    // ros初始化
    ros::init(argc, argv, "odom2posenode");
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe<sensor_msgs::PointCloud2>("/pointcloud_in", 10, cloud_callback);
    // mavros视觉定位发布
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/trasformed/pointcloud", 10);
    rotMatrix << 0.939681, 0.0, 0.342052, 0.0,
    0.0, 1.0, 0.0, 0.0,
    -0.342052, 0.0, 0.939681, 0.1,
    0.0, 0.0, 0.0, 1.0;
    ros::spin();
}
