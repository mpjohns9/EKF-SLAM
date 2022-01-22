/// \file
/// \brief Node that can be used as simulator and visualizer.
///
/// PARAMETERS:
///     x0 (double): starting x-coordinate of robot
///     y0 (double): starting y-coordinate of robot
///     theta0 (double): starting orientation of robot
///     obs_x (std::vector<double>): list of x-coordinates for obstacles
///     obs_y (std::vector<double>): list of x-coordinates for obstacles
///     radius (double): radius of obstacles
///     height (double): height of obstacles
/// PUBLISHES:
///     timestep (std_msgs/UInt64): publishes timestep of simulation
///     red/joint_states (sensor_msgs/JointState): publishes joint states
///     obstacles (visualization_msgs/MarkerArray): publishes array of cylindrical markers
/// SERVICES:
///     reset (Empty): resets the timestep to 0 and robot to initial position
///     teleport (nusim/Teleport): teleports robot to x, y, theta position


#include "ros/ros.h"
#include <std_msgs/UInt64.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "nusim/Teleport.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/console.h>

static int timestep;
static int rate;
static double x_0, y_0, theta_0;
static double x, y, theta;
static std::vector<double> obs_x;
static std::vector<double> obs_y;
static double radius;
static double height;

/// \brief callback for reset service
///
/// resets timestep to 0 and robot to initial position
bool reset_callback(std_srvs::TriggerRequest & request, std_srvs::TriggerResponse & response)
{
    timestep = 0;
    x = 0.0;
    y = 0.0;
    theta = 0.0;
    return true;
}


/// \brief callback for teleport service
///
/// teleports robot to x, y, theta position
bool teleport_callback(nusim::Teleport::Request & request, nusim::Teleport::Response & response)
{
    x = request.x;
    y = request.y;
    theta = request.theta;
    return true;
}

int main(int argc, char * argv[])
{

    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh_prv("~");
    ros::NodeHandle nh;
    ros::Rate r(nh_prv.param("rate", rate, 500));

    nh_prv.param("x0", x_0, 0.0);
    nh_prv.param("y0", y_0, 0.0);
    nh_prv.param("theta0", theta_0, 0.0);

    nh_prv.getParam("obstacles/obs_x", obs_x);
    nh_prv.getParam("obstacles/obs_y", obs_y);
    nh_prv.getParam("obstacles/radius", radius);
    nh_prv.getParam("obstacles/height", height);

    x = x_0;
    y = y_0;
    theta = theta_0;

    std_msgs::UInt64 step;

    ros::Publisher pub_step = nh_prv.advertise<std_msgs::UInt64>("timestep", 1000);
    ros::Publisher pub_joints = nh.advertise<sensor_msgs::JointState>("red/joint_states", 1000);
    ros::Publisher marker_pub = nh_prv.advertise<visualization_msgs::MarkerArray>("obstacles", 1, true);

    sensor_msgs::JointState js;

    ros::ServiceServer reset = nh_prv.advertiseService("reset", reset_callback);
    ros::ServiceServer teleport = nh_prv.advertiseService("teleport", teleport_callback);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transform;
    tf2::Quaternion q;

    visualization_msgs::MarkerArray ma;

    int i;
    ma.markers.resize(obs_x.size());
    for (i=0;i<obs_x.size();i++)
    {
        // visualization_msgs::Marker marker;

        //set header and timestamp
        ma.markers[i].header.frame_id = "world";
        ma.markers[i].header.stamp = ros::Time::now();

        //set id diff for each marker
        ma.markers[i].id = i;

        //set color and action
        ma.markers[i].type = visualization_msgs::Marker::CYLINDER;
        ma.markers[i].action = visualization_msgs::Marker::ADD;

        //set pose of marker
        ma.markers[i].pose.position.x = obs_x.at(i);
        ma.markers[i].pose.position.y = obs_y.at(i);
        ma.markers[i].pose.position.z = 0;
        ma.markers[i].pose.orientation.x = 0;
        ma.markers[i].pose.orientation.y = 0;
        ma.markers[i].pose.orientation.z = 0;
        ma.markers[i].pose.orientation.w = 1;

        //set size
        ma.markers[i].scale.x = 2*radius;
        ma.markers[i].scale.y = 2*radius;
        ma.markers[i].scale.z = height;


        //set color
        ma.markers[i].color.r = 1.0;
        ma.markers[i].color.g = 0.0;
        ma.markers[i].color.b = 0.0;
        ma.markers[i].color.a = 1.0;

        // ma.markers[i].lifetime = ros::Duration();


        // ma.markers.push_back(marker);
    }

    marker_pub.publish(ma);

    
    timestep = 0;
    while(ros::ok())
    {
        ROS_DEBUG("hello");
        transform.header.stamp = ros::Time::now();

        transform.header.frame_id = "world";
        transform.child_frame_id = "red_base_footprint";

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

        step.data = timestep;
        pub_step.publish(step);

        js.name = {"red_wheel_left_joint", "red_wheel_right_joint"};
        js.position = {0.0, 0.0};
        pub_joints.publish(js);
        timestep++;
        ros::spinOnce();
        r.sleep();
    }
    return 0;
    
}