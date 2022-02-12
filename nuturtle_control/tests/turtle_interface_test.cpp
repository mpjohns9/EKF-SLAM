#include "ros/ros.h"
#include "catch_ros/catch.hpp"
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"


/// \file
/// \brief Test file for turtle_interface node ROS API

// wheel velocities
static auto lwheel_vel = 0.0;
static auto rwheel_vel = 0.0;

// wheel positions
static auto lwheel_pos = 0.0;
static auto rwheel_pos = 0.0;

void wheelCallback(const nuturtlebot_msgs::WheelCommands & msg)
{
    lwheel_vel = msg.left_velocity;
    rwheel_vel = msg.right_velocity;
}

void jointCallback(const sensor_msgs::JointState & msg)
{
    lwheel_pos = msg.position.at(0);
    rwheel_pos = msg.position.at(1);
}

TEST_CASE("turtle_interface tests", "[turtle_interface]") {

    SECTION("cmd_vel translation test") {
        ros::NodeHandle nh;
        ros::Subscriber wheel_sub = nh.subscribe("wheel_cmd", 1000, wheelCallback);
        ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

        geometry_msgs::Twist t;
        t.linear.x = 1.0;
        t.angular.z = 0.0;

        for (int i=0;i<1000000;i++) {

            vel_pub.publish(t);
            ros::spinOnce();
        }

        CHECK(lwheel_vel == Approx(256));
        CHECK(rwheel_vel == Approx(256));
    }

    SECTION("cmd_vel rotation test") {
        ros::Duration(3.0).sleep();
        ros::NodeHandle nh;
        ros::Subscriber wheel_sub = nh.subscribe("wheel_cmd", 1000, wheelCallback);
        ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

        geometry_msgs::Twist t;
        t.linear.x = 0.0;
        t.angular.z = 1.0;

        for (int i=0;i<1000000;i++) {

            vel_pub.publish(t);
            ros::spinOnce();
        }

        CHECK(lwheel_vel == Approx(-101));
        CHECK(rwheel_vel == Approx(101));
    }
    SECTION("sensor_data test") {
        ros::Duration(3.0).sleep();
        ros::NodeHandle nh;
        ros::Subscriber joint_sub = nh.subscribe("red/joint_states", 1000, jointCallback);
        ros::Publisher sensor_pub = nh.advertise<nuturtlebot_msgs::SensorData>("sensor_data", 1000);

        nuturtlebot_msgs::SensorData sd;
        sd.left_encoder = 1;
        sd.right_encoder = 1;

        for (int i=0;i<1000000;i++) {

            sensor_pub.publish(sd);
            ros::spinOnce();

        }

        CHECK(lwheel_pos == Approx(0.00153398));
        CHECK(rwheel_pos == Approx(0.00153398));
    }

}



