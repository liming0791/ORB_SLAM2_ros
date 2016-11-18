#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_datatypes.h"

#include <sstream>
#include <boost/function.hpp>
#include <mutex>

#include "orb_slam2/GetIMUPoseWorldService.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
ros::Publisher *pPub = NULL;
double ax = 0, ay = 0, az = 0;
double q0 = 1, q1 = 0, q2 = 0, q3 = 0;

void GetIMUPoseWorld(const sensor_msgs::Imu &imu_data)
{
    ax = 0.9*ax + 0.1*imu_data.linear_acceleration.x;
    ay = 0.9*ay + 0.1*imu_data.linear_acceleration.y;
    az = 0.9*az + 0.1*imu_data.linear_acceleration.z;

    double r = atan2(ay, az) ;
    double p = atan2(ax, sqrt(az*az+ay*ay)) ;
    double y = 0;

    geometry_msgs::PoseStamped pose;
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p, y);
    pose.header.frame_id = "world";

    {
        q0 = pose.pose.orientation.w;
        q1 = pose.pose.orientation.x;
        q2 = pose.pose.orientation.y;
        q3 = pose.pose.orientation.z;
    }

    pPub->publish(pose);
}

bool GetIMUPoseWorldService(orb_slam2::GetIMUPoseWorldService::Request &req,
        orb_slam2::GetIMUPoseWorldService::Response &res)
{
    res.w = q0;
    res.x = q1;
    res.y = q2;
    res.z = q3;
    return true;
}

int main(int argc, char **argv)
{
    // Init ros
    ros::init(argc, argv, "imu_world_pose");
    ros::NodeHandle n;
    
    ros::Publisher pub = 
        n.advertise<geometry_msgs::PoseStamped>("/imu_world_pose/pose", 5);
    pPub = &pub;

    ros::Subscriber sub = n.subscribe("/adafruit_imu/imu_data", 5, GetIMUPoseWorld);

    ros::ServiceServer service = n.advertiseService("GetIMUPoseWorldService", GetIMUPoseWorldService);

    ros::spin();

    return 0;
}
