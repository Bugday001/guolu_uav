/**
 * 将cloud变换到world坐标系，提供给fast-planner
 * 
 * 2023.01.14
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

class SegMapROSWraper
{
private:
    ros::NodeHandle m_nh;
    ros::Publisher m_globalcloudPub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *m_pointCloudSub;
    tf::MessageFilter<sensor_msgs::PointCloud2> *m_tfPointCloudSub;
    tf::TransformListener m_tfListener;

public:
    SegMapROSWraper()
        : m_nh("~")
    {
        std::string pointCloud_tag = "/3Dlidar16_scan";
        m_nh.getParam("pointCloud_tag", pointCloud_tag);
        // std::cout<<"!!!!!!!!!!!!!!!"<<pointCloud_tag<<std::endl;
        m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(m_nh, pointCloud_tag, 100);
        m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2>(*m_pointCloudSub, m_tfListener, "/world", 100);
        m_tfPointCloudSub->registerCallback(boost::bind(&SegMapROSWraper::insertCloudCallback, this, _1));
        m_globalcloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("/global_map", 2, true);
    }

    ~SegMapROSWraper()
    {
        delete m_pointCloudSub;
        delete m_tfPointCloudSub;
    }

    void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud)
    {
        //不要使用XYZL，读不到label信息会报warning
        pcl::PointCloud<pcl::PointXYZ> pc;
        pcl::PointCloud<pcl::PointXYZ> pc_global;
        // pcl::PCLPointCloud2 pcl_pc2;
        // pcl_conversions::toPCL(*cloud, pcl_pc2);//moveToPCL
        // pcl::fromPCLPointCloud2(pcl_pc2, pc);
        pcl::fromROSMsg(*cloud, pc);

        tf::StampedTransform sensorToWorldTf;
        try
        {
            m_tfListener.lookupTransform("/world", cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
            return;
        }

        Eigen::Matrix4f sensorToWorld;
        pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
        pcl::transformPointCloud(pc, pc_global, sensorToWorld);
        // std::cout<< sensorToWorld <<std::endl;
        sensor_msgs::PointCloud2 map_cloud;
        pcl::toROSMsg(pc_global, map_cloud);
        map_cloud.header.stamp = ros::Time::now();
        map_cloud.header.frame_id = "map";
        m_globalcloudPub.publish(map_cloud);
    }
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "colored");

    SegMapROSWraper SM;

    ros::spin();
    return 0;
}