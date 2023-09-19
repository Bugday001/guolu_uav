/**
 * 判断是否能够碰撞
 * 
 * 定时发布/collision_warning给fastplanner的topo_replan_fsm
 * 数据定义:
 *  0: 安全，正常飞行
 *  1: 危险，立刻悬停，将fastplanner状态机设置为WAIT_TARGET
 * 
 * 2023.06.30
 */
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/filters/crop_box.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <mavros_msgs/PlayTuneV2.h>

class avoidance
{
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr map{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr after_CropBox{new pcl::PointCloud<pcl::PointXYZ>};
    ros::NodeHandle nh;
    ros::Publisher part_map_pub, warning_pub,buzzer_pub;

    mavros_msgs::PlayTuneV2 buzzer_tune;
    std_msgs::UInt8 isCollision;

    ros::Timer timer;
    int threshold = 50;
    Eigen::Vector3f xyz;
    Eigen::Vector3f ea;
    ros::Subscriber map_sub, odom_sub;
    pcl::CropBox<pcl::PointXYZ> clipper;
public:
    avoidance(): nh()
    {
        ROS_INFO("init");
        map_sub = nh.subscribe<sensor_msgs::PointCloud2>("/robot/dlio/odom_node/pointcloud/deskewed", 10, &avoidance::map_callback, this);
        odom_sub = nh.subscribe<nav_msgs::Odometry>("/robot/dlio/odom_node/odom", 10, &avoidance::odom_callback, this);
        part_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/avoidance_cp", 2, true);
        warning_pub = nh.advertise<std_msgs::UInt8>("/collision_warning", 20);
        buzzer_pub = nh.advertise<mavros_msgs::PlayTuneV2>("/mavros/play_tune", 20);

        buzzer_tune.format = 1;
        buzzer_tune.tune = ">e16e8e16r16c16e8g4<g4";
        isCollision.data = 0;  //0:safe, 1:stop

        bool negative = false;
        float crop_size_ = 1.5; // 距离-1到1
        clipper.setMin(Eigen::Vector4f(-crop_size_, -crop_size_, -0.2, 1.0));
        clipper.setMax(Eigen::Vector4f(crop_size_, crop_size_, crop_size_, 1.0));
        clipper.setNegative(negative);
        timer= nh.createTimer(ros::Duration(0.05), &avoidance::timer_callback, this);//定时器
    }

    void timer_callback(const ros::TimerEvent& event)
    {
        // clip
        clipper.setTranslation(xyz);
        clipper.setRotation(ea);
        clipper.setInputCloud(map);
        clipper.filter(*after_CropBox);
        if (after_CropBox->size() > threshold)
        {
            ROS_WARN("!!!!!!!!!!!!!!!!!!!!COLLISION WARNING!!!!!!!!!!!!!!!!!!\n");
            buzzer_pub.publish(buzzer_tune);
            isCollision.data = 1;
        }
        else 
        {
            isCollision.data = 0;
        }
        warning_pub.publish(isCollision);
        sensor_msgs::PointCloud2 map_cloud;
        pcl::toROSMsg(*after_CropBox, map_cloud);
        map_cloud.header.stamp = ros::Time::now();
        map_cloud.header.frame_id = "map";
        part_map_pub.publish(map_cloud);
    }

    void map_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud)
    {
        pcl::fromROSMsg(*cloud, *map);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        xyz(0) = msg->pose.pose.position.x;
        xyz(1) = msg->pose.pose.position.y;
        xyz(2) = msg->pose.pose.position.z;
        Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Eigen::Matrix3d rx = q.toRotationMatrix();
        ea = rx.eulerAngles(0,1,2).cast<float>();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_node");
    avoidance avoider;
    ros::spin();
    return 0;
}