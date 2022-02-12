#include "ros/ros.h"
#include "catch_ros/catch.hpp"
#include "nuturtle_control/SetPose.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>


/// \file
/// \brief Test file for odometry node ROS API

TEST_CASE("odometry tests", "[turtle_interface]") {

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<nuturtle_control::SetPose>("set_pose");

    ros::service::waitForService("set_pose");

    nuturtle_control::SetPose srv;
    srv.request.x = 0.0;
    srv.request.y = 0.0;
    srv.request.theta = 0.0;

    CHECK(client.call(srv));

}

TEST_CASE("tf2 listener test", "[turtle_interface]") {

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tf_listener(buffer);

    geometry_msgs::TransformStamped tf_stamped;

    for (int i=0;i<100;i++)
    {
        try {
            tf_stamped = buffer.lookupTransform("odom", "blue_base_footprint", ros::Time(0));
        }

        catch (tf2::TransformException &e) {
            ROS_WARN("%s",e.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }

    CHECK(tf_stamped.transform.translation.x == Approx(0.0));
    CHECK(tf_stamped.transform.translation.y == Approx(0.0));

}