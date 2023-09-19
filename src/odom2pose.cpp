#include <iostream>
#include <iomanip>
#include <fstream>
#include <time.h>
#include <cmath>
#include <tf/tf.h>
#include <Eigen/Core>
//ros相关节点
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::PoseStamped  visionPose;
geometry_msgs::PoseStamped  visionPosition;
geometry_msgs::Twist visionVelocity;
ros::Publisher posePub, velPub, visionPosPub;
ros::Timer timer;


void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
    visionPose.header.stamp = ros::Time::now();
    visionPose.header.frame_id = "map";
    visionPose.pose.position.x = msg -> pose.pose.position.x;
    visionPose.pose.position.y = msg -> pose.pose.position.y;
    visionPose.pose.position.z = msg -> pose.pose.position.z;
    visionPose.pose.orientation.x = msg -> pose.pose.orientation.x;
    visionPose.pose.orientation.y = msg -> pose.pose.orientation.y;
    visionPose.pose.orientation.z = msg -> pose.pose.orientation.z;
    visionPose.pose.orientation.w = msg -> pose.pose.orientation.w;

    visionPosition.pose.position.x = msg -> pose.pose.position.x;
    visionPosition.pose.position.y = msg -> pose.pose.position.y;
    visionPosition.pose.position.z = msg -> pose.pose.position.z;
    //速度用的似乎是机体系
    // std::cout << " " << visionPosition.pose.position.x << "  " << visionPosition.pose.position.y << "  " << visionPosition.pose.position.z << std::endl;
    visionVelocity.linear.x = msg -> twist.twist.linear.x;
    visionVelocity.linear.y = msg -> twist.twist.linear.y;
    visionVelocity.linear.z = msg -> twist.twist.linear.z;
    // std::cout << " " << visionVelocity.linear.x << "  " << visionVelocity.linear.y << "  " << visionVelocity.linear.z << std::endl;
}


void timer_callback(const ros::TimerEvent& event)
{
    visionPosPub.publish(visionPose);
}

int main(int argc, char** argv){


    //ros初始化
    ros::init(argc, argv, "odom2posenode");
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odometry/imu",10, odom_callback);
    //mavros视觉定位发布
    visionPosPub = nh.advertise<geometry_msgs::PoseStamped> ("/mavros/vision_pose/pose", 10);  
    timer= nh.createTimer(ros::Duration(0.05), timer_callback);//定时器

    ros::spin();
}





