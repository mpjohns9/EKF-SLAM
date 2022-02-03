/// \file
/// \brief publishes odometry messages and transform

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"

static auto body_id = "";
static auto odom_id = "";

static auto x = 0;
static auto y = 0;
static auto theta = 0;

static auto lwheel_vel = 0;
static auto rwheel_vel = 0;

static auto lwheel_pos = 0;
static auto rwheel_pos = 0;

void jointCallback(const sensor_msgs::JointState & msg)
{
    lwheel_vel = msg.velocity[0];
    rwheel_vel = msg.velocity[1];
    
    lwheel_pos = msg.position[0];
    rwheel_pos = msg.position[1];
}

bool poseCallback(geometry_msgs::Pose::Request & request, geometry_msgs::Pose::Response & response)
{
    x = request.position.x;
    y = request.position.y;
    theta = request.orientation.theta;
    return true;
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle nh_prv("~");
    ros::NodeHandle nh;
    ros::Rate r(nh_prv.param("rate", rate, 500));

    if (!nh.hasParam("body_id"))
    {
        ROS_ERROR_STREAM("No param named 'body_id'.");
        return 1;
    }
    else
    {
        nh.getParam("body_id", body_id);
    }

    if (!nh.hasParam("wheel_left"))
    {
        ROS_ERROR_STREAM("No param named 'wheel_left'.");
        return 1;
    }
    else
    {
        nh.getParam("wheel_left", wheel_left);
    }

    if (!nh.hasParam("wheel_right"))
    {
        ROS_ERROR_STREAM("No param named 'wheel_right'.");
        return 1;
    }
    else
    {
        nh.getParam("wheel_right", wheel_right);
    }

    nh.param("odom_id", odom_id, "odom");

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);

    ros::Subscriber joint_sub = nh.subscribe("joint_states", 1000, jointCallback);

    ros::ServiceServer set_pose = nh.advertiseService("set_pose", poseCallback);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transform;
    tf2::Quaternion q;

    while(ros::ok())
    {
        nav_msgs::Odometry odom;

        transform.header.stamp = ros::Time::now();

        transform.header.frame_id = odom_id;
        transform.child_frame_id = body_id;

        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta);

        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        br.sendTransform(transform);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}