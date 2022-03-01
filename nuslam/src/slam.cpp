// /// \file
// /// \brief publishes odometry messages and transform

// /// PARAMETERS:
// ///     body_id: name of robot body frame
// ///     odom_id: name of odometry frame
// ///     wheel_left: name of left wheel joint
// ///     wheel_right: name of right wheel joint
// ///
// /// PUBLISHES: 
// ///     odom_pub (nav_msgs/Odometry): publishes to odom
// ///
// /// SUBSCRIBES:
// ///     joint_sub (joint_states): subscribes to joint_states
// ///
// /// SERVICES: 
// ///     set_pose (geometry_msgs/Pose): provides configuration of robot

// #include "ros/ros.h"
// #include <sensor_msgs/JointState.h>
// #include <nav_msgs/Odometry.h>
// #include <geometry_msgs/PoseWithCovariance.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include "nuturtle_control/SetPose.h"
// #include "turtlelib/rigid2d.hpp"
// #include "turtlelib/diff_drive.hpp"

// turtlelib::diffDrive dd;

// static auto rate = 0;

// static std::string body_id = "";
// static std::string odom_id = "";
// static std::string wheel_left = "";
// static std::string wheel_right = "";

// static auto x = 0.0;
// static auto y = 0.0;
// static auto theta = 0.0;

// static auto lwheel_vel = 0.0;
// static auto rwheel_vel = 0.0;

// static auto lwheel_pos = 0.0;
// static auto rwheel_pos = 0.0;

// /// \brief callback for the joint_states subscriber
// /// \param msg - sensor_msgs/JointState message obj
// void jointCallback(const sensor_msgs::JointState & msg)
// {
//     // ROS_ERROR_STREAM("VELOCITY " << msg.velocity[0] << " " << msg.velocity[1]);
//     lwheel_vel = msg.velocity.at(0);
//     rwheel_vel = msg.velocity.at(1);
    
//     // ROS_ERROR_STREAM("POSITION " << msg.position[0] << msg.position[1]);
//     lwheel_pos = msg.position.at(0);
//     rwheel_pos = msg.position.at(1);

//     turtlelib::WheelPos pos {lwheel_pos, rwheel_pos};
//     turtlelib::Config c;

//     c = dd.fwd_kin(pos);

//     x = c.x;
//     y = c.y;
//     theta = c.ang;

//     turtlelib::WheelVel v;
//     v.l_vel = msg.velocity.at(0);
//     v.r_vel = msg.velocity.at(1);

//     dd = turtlelib::diffDrive(c, pos, v);
// }

// /// \brief callback for set pose service
// bool poseCallback(nuturtle_control::SetPose::Request & request, nuturtle_control::SetPose::Response & response)
// {
//     x = request.x;
//     y = request.y;
//     theta = request.theta;

//     turtlelib::Config c = {theta, x, y};
//     dd = turtlelib::diffDrive(c);
//     return true;
// }

// int main(int argc, char * argv[])
// {
//     ros::init(argc, argv, "odometry");
//     ros::NodeHandle nh_prv("~");
//     ros::NodeHandle nh;

//     nh_prv.param("rate", rate, 500);
//     ros::Rate r(rate);

//     if (!nh.hasParam("body_id"))
//     {
//         ROS_ERROR_STREAM("No param named 'body_id'.");
//         return 1;
//     }
//     else
//     {
//         nh.getParam("body_id", body_id);
//         // ROS_ERROR_STREAM("BODY ID " << body_id);
//     }

//     if (!nh.hasParam("wheel_left"))
//     {
//         ROS_ERROR_STREAM("No param named 'wheel_left'.");
//         return 1;
//     }
//     else
//     {
//         nh.getParam("wheel_left", wheel_left);
//         // ROS_ERROR_STREAM("WHEEL LEFT ID " << wheel_left);
//     }

//     if (!nh.hasParam("wheel_right"))
//     {
//         ROS_ERROR_STREAM("No param named 'wheel_right'.");
//         return 1;
//     }
//     else
//     {
//         nh.getParam("wheel_right", wheel_right);
//         // ROS_ERROR_STREAM("WHEEL RIGHT ID " << wheel_right);
//     }

//     nh.param<std::string>("odom_id", odom_id, "odom");

//     ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);

//     ros::Subscriber joint_sub = nh.subscribe("joint_states", 1000, jointCallback);

//     ros::ServiceServer set_pose = nh.advertiseService("set_pose", poseCallback);

//     static tf2_ros::TransformBroadcaster br;
//     geometry_msgs::TransformStamped transform;
//     tf2::Quaternion q;

//     while(ros::ok())
//     {
//         geometry_msgs::PoseWithCovariance p;
//         p.pose.position.x = x;
//         p.pose.position.y = y;
//         p.pose.position.z = 0.0;

//         // geometry_msgs::TwistWithCovariance t;
//         // t.position.x = x;
//         // t.position.y = y;
//         // t.position.z = 0.0;

//         nav_msgs::Odometry odom;
//         /// HOW TO GET THE TWIST?
//         odom.header.stamp = ros::Time::now();
//         odom.header.frame_id = body_id;
//         odom.child_frame_id = odom_id;
//         odom.pose = p;
//         odom_pub.publish(odom);

//         transform.header.stamp = ros::Time::now();

//         transform.header.frame_id = odom_id;
//         transform.child_frame_id = "blue_base_footprint";

//         transform.transform.translation.x = x;
//         transform.transform.translation.y = y;
//         transform.transform.translation.z = 0.0;

//         tf2::Quaternion q;
//         q.setRPY(0, 0, theta);

//         transform.transform.rotation.x = q.x();
//         transform.transform.rotation.y = q.y();
//         transform.transform.rotation.z = q.z();
//         transform.transform.rotation.w = q.w();

//         br.sendTransform(transform);
//         // ROS_ERROR_STREAM("THE END");
//         ros::spinOnce();
//         r.sleep();
//     }

//     return 0;
// }