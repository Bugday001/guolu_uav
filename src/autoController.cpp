#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/RCIn.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <sensor_msgs/BatteryState.h>
#include <std_srvs/SetBool.h>
#include <tf/transformations.h>
#include <yaml/yaml.h>

const double THRESHOLD = 0.3;
bool Resume = false;

class SendPath
{
public:
    SendPath()
    {
        // Initialize the node
        ros::init(argc, argv, "send_path");
        ros::NodeHandle nh;

        // Subscribe to relevant topics
        position_sub = nh.subscribe("mavros/local_position/pose", 10, &SendPath::position_callback, this);
        battery_sub = nh.subscribe("mavros/battery", 10, &SendPath::battery_callback, this);
        rc_sub = nh.subscribe("mavros/rc/in", 10, &SendPath::rc_callback, this);

        // Set up the waypoint publisher
        waypoint_pub = nh.advertise<nav_msgs::Path>("waypoint_generator/waypoints", 10);
        transformed_pose_pub = nh.advertise<nav_msgs::PoseStamped>("mavros/local_position/pose_transformed", 10);

        // Set up the service for triggering the path
        path_srv = nh.advertiseService("planning/request_target", &SendPath::send_target_point, this);

        // Set up the service for landing
        land_srv = nh.serviceProxy<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land", 10);

        // Set up the battery threshold
        battery_threshold_sub = nh.subscribe("mavros/battery", 10, &SendPath::battery_threshold_callback, this);

        // Load the path and landing positions from a file
        load_path_from_file();

        // Set up the state variables
        state = FLY;
        last_update_time = rospy::Time::now();
        cur_position = Point();
        i = 0;
        target_point = pathList[0];
        island = false;
        is_offboard = true;

        // Set up the log file
        log_file = std::ofstream(ros::get_name() + ".log", std::ios::app);
        log_file << "Logging started at " << rospy::Time::now().to_sec() << std::endl;

        // Set up the record file
        record_file = std::ifstream(ros::package::getPath("px4_offboard") + "/params/record.yaml", std::ios::in);
        record_data = yaml::load(record_file);
        i = record_data["index"];
        target_point = pathList[i];

        // Set up the transformation matrix
        // TODO: Load the transformation matrix from a file

        // Start the main loop
        ros::spin();
    }

private:
    // Callback functions
    void position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        cur_position = msg->pose.position;

        // Check if we reached the next waypoint
        if (i < pathList.size() && reach_goal())
        {
            i++;
            target_point = pathList[i];

            // If we reached the last waypoint, mark the state as LAND
            // If we reached the last waypoint, mark the state as LAND
            if (i == pathList.size() - 1)
            {
                state = LAND;
                log_file << "Reached the last waypoint, entering land mode at " << rospy::Time::now().to_sec() << std::endl;
            }

            // Publish the transformed pose for debugging purposes
            // TODO: Implement this

            // Publish the path for Fast-Planner
            nav_msgs::Path path;
            path.header.stamp = rospy::Time::now();
            path.header.frame_id = "world";

            for (int j = 0; j < pathList.size(); j++)
            {
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = rospy::Time::now();
                pose.header.frame_id = "world";
                pose.pose.position.x = pathList[j][0];
                pose.pose.position.y = pathList[j][1];
                pose.pose.position.z = pathList[j][2];
                pose.pose.orientation = quaternion_from_euler(0, 0, pathList[j][3]);
                path.poses.push_back(pose);
            }
            waypoint_pub.publish(path);
        }
    }

    void battery_callback(const sensor_msgs::BatteryState::ConstPtr &msg)
    {
        if (msg->voltage < 18.5)
        {
            state = LAND;
            log_file << "Low battery, entering land mode at " << rospy::Time::now().to_sec() << std::endl;
        }
    }

    void rc_callback(const mavros_msgs::RCIn::ConstPtr &msg)
    {
        // Check if the RC throttle is set to 1050 and the yaw is set to 1950
        if (rc_history.channels[7] == 1050 && msg->channels[7] == 1950 && state == FLY)
        {
            state = LAND;
            log_file << "RC command received, entering land mode at " << rospy::Time::now().to_sec() << std::endl;
        }
        rc_history = msg;
    }

    void battery_threshold_callback(const sensor_msgs::BatteryState::ConstPtr &msg)
    {
        if (msg->voltage < 18.5)
        {
            state = LAND;
            log_file << "Low battery, entering land mode at " << rospy::Time::now().to_sec() << std::endl;
        }
    }

    bool reach_goal()
    {
        return std::abs(cur_position.x - target_point[0]) < THRESHOLD &&
                std::abs(cur_position.y - target_point[1]) < THRESHOLD &&
                std::abs(cur_position.z - target_point[2]) < THRESHOLD;
    }

    void load_path_from_file()
    {
        // TODO: Implement this
    }

    // Function to send the target point to Fast-Planner
    bool send_target_point(std_srvs::SetBool::Request & request, std_srvs::SetBool::Response & response)
    {
        if (state == FLY)
        {
            response.success = true;
            response.message = "Sending target point";

            // TODO: Implement this
        }
        else
        {
            response.success = false;
            response.message = "Cannot send target point in land mode";
        }

        return response.success;
    }
};

int main(int argc, char **argv)
{
    SendPath sendPath;
    return 0;
}