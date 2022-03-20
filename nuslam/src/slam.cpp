/// \file
/// \brief EKF SLAM implementation
/// publishes odometry messages and transform for SLAM

/// PARAMETERS:
///     body_id: name of robot body frame
///     odom_id: name of odometry frame
///     wheel_left: name of left wheel joint
///     wheel_right: name of right wheel joint
///     radius: obstacle radius
///     height: obstacle height
///     range_max: maximium range of laser scan
///
/// PUBLISHES: 
///     odom_pub (nav_msgs/Odometry): publishes to odom
///     marker_pub (visualization_msgs/MarkerArray): publishes markers from SLAM
///     path_pub (nav_msgs/Path): publishes path for SLAM robot
///
/// SUBSCRIBES:
///     joint_sub (joint_states): subscribes to joint_states
///     landmark_sub (landmarks): subscribes to landmarks to get obstacles position
///
/// SERVICES: 
///     set_pose (geometry_msgs/Pose): provides configuration of robot

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include "nuturtle_control/SetPose.h"
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"

turtlelib::diffDrive dd;
turtlelib::EKF e(100);

static auto rate = 0;

static std::string body_id = "";
static std::string odom_id = "";
static std::string wheel_left = "";
static std::string wheel_right = "";

static auto x = 0.0;
static auto y = 0.0;
static auto theta = 0.0;

turtlelib::Twist2D u;

static auto lwheel_vel = 0.0;
static auto rwheel_vel = 0.0;

static auto lwheel_pos = 0.0;
static auto rwheel_pos = 0.0;

static auto landmark_flag = true;
static auto marker_flag = false;

static std::vector<double> obstacles_slam;

visualization_msgs::MarkerArray ma;

static auto radius = 0.0;
static auto height = 0.0;
static auto range_max = 0.0;

nav_msgs::Path path;
geometry_msgs::PoseStamped ps;


/// \brief callback for the joint_states subscriber
/// \param msg - sensor_msgs/JointState message obj
void jointCallback(const sensor_msgs::JointState & msg)
{
    // ROS_ERROR_STREAM("VELOCITY " << msg.velocity[0] << " " << msg.velocity[1]);
    lwheel_vel = msg.velocity.at(0);
    rwheel_vel = msg.velocity.at(1);
    
    // ROS_ERROR_STREAM("POSITION " << msg.position[0] << msg.position[1]);
    lwheel_pos = msg.position.at(0);
    rwheel_pos = msg.position.at(1);

    turtlelib::WheelPos pos {lwheel_pos, rwheel_pos};
    turtlelib::Config c;

    std::tie(c, u) = dd.fwd_kin(pos);

    x = c.x;
    y = c.y;
    theta = c.ang;
    // ROS_ERROR_STREAM("POSL: " << pos.l_pos);
    // ROS_ERROR_STREAM("POSR: " << pos.r_pos);
    // ROS_ERROR_STREAM("X: " << x);
    // ROS_ERROR_STREAM("Y: " << y);
    // ROS_ERROR_STREAM("THETA: " << theta);
    // ROS_ERROR_STREAM("TWIST: " << u);

    turtlelib::WheelVel v;
    v.l_vel = msg.velocity.at(0);
    v.r_vel = msg.velocity.at(1);

    dd = turtlelib::diffDrive(c, pos, v);
}

/// \brief callback for the landmarks subscriber
/// \param msg - visualization_msgs/MarkerArray message obj
void landmarkCallback(const visualization_msgs::MarkerArray & msg)
{
    int n = msg.markers.size();

    std::vector<double> obs_x(n);
    std::vector<double> obs_y(n);
    // std::vector<double> z_sensor(n*2);
    turtlelib::WheelPos pos {lwheel_pos, rwheel_pos};
    u = std::get<1>(dd.fwd_kin(pos));

    // int index = 0;
    for (int i=0; i<n; i++)
    {
        double x = msg.markers.at(i).pose.position.x;
        double y = msg.markers.at(i).pose.position.y;

        obs_x.at(i) = x;
        obs_y.at(i) = y;

        if (e.check_known_obs(x, y))
        {
            // ROS_ERROR_STREAM("NEW OBSTACLE DETECTED");
            landmark_flag = true;
        }

        // z_sensor.at(index) = x;
        // z_sensor.at(index + 1) = y;
        // index += 2;
        // ROS_ERROR_STREAM("TWIST: " << u);
    }

    if (landmark_flag)
    {
        // ROS_ERROR_STREAM("INITIALIZING LANDMARKS");
        e.initialize_landmarks(obs_x, obs_y);
        landmark_flag = false;
    }

    for (int j=0; j<n; j++)
    {
        // ROS_ERROR_STREAM("OBSTACLE " << j);
        // ROS_ERROR_STREAM("X: " << obs_x.at(j));
        // ROS_ERROR_STREAM("Y: " << obs_y.at(j));
        // ROS_ERROR_STREAM("TWIST: " << u);
        e.predict(u);
        e.update(j, obs_x.at(j), obs_y.at(j));
        obstacles_slam = e.obs_vec();
    }
}

/// \brief callback for set pose service
bool poseCallback(nuturtle_control::SetPose::Request & request, nuturtle_control::SetPose::Response &)
{
    x = request.x;
    y = request.y;
    theta = request.theta;

    turtlelib::Config c = {theta, x, y};
    dd = turtlelib::diffDrive(c);
    return true;
}

/// \brief callback for the marker timer
void markerCallback(const ros::TimerEvent&)
{
    int non_zero = 0;
    for (int i=0; i<int(obstacles_slam.size()/2); i++)
    {
        double check_x = obstacles_slam.at(2*i);
        double check_y = obstacles_slam.at((2*i)+1);

        // ROS_ERROR_STREAM("X " << check_x);
        // ROS_ERROR_STREAM("Y " << check_y);

        if (check_x != 0.0 && check_y != 0.0)
        {
            non_zero += 1;
        }
    }
    // ROS_ERROR_STREAM("COUNT" << non_zero);
    ma.markers.resize(non_zero);
    // ROS_ERROR_STREAM("SIZE: " << int(obstacles_slam.size()));
    for (int i=0;i<int(non_zero);i++)
    {
        // ROS_ERROR_STREAM("obstacles_slam: " << obstacles_slam);
        // ROS_ERROR_STREAM("id: " << i);
        // turtlelib::Vector2D vec{x, y};
        // turtlelib::Transform2D Twb(vec, theta);
        // turtlelib::Transform2D Tbw = Twb.inv();

        double x_obstacle = obstacles_slam.at(2*i);
        double y_obstacle = obstacles_slam.at((2*i)+1);

        // if (x_obstacle == 0 && y_obstacle == 0)
        // {
        //     continue;
        // }

        // turtlelib::Vector2D obs_vec {x_obstacle, y_obstacle};
        // turtlelib::Vector2D obs_vec_b = Tbw(obs_vec);

        // double obs_theta = atan2(obs_vec_b.x, obs_vec_b.y) + obs_noise;
        double dx = x_obstacle-e.config().x;
        double dy = y_obstacle-e.config().y;
        double r = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

        // double distance = std::sqrt(std::pow(x_obstacle - x, 2) + std::pow(y_obstacle - y, 2));
        // visualization_msgs::Marker marker;

        //set header and timestamp
        ma.markers[i].header.frame_id = "map";
        ma.markers[i].header.stamp = ros::Time::now();

        //set id diff for each marker
        ma.markers[i].id = i;

        //set color and action
        ma.markers[i].type = visualization_msgs::Marker::CYLINDER;
        // ROS_ERROR_STREAM("r (Obstacle " << i << "): " << r);
        // ROS_ERROR_STREAM("range_max (Obstacle " << i << "): " << range_max);
        if (r <= range_max)
        {
            ma.markers[i].action = visualization_msgs::Marker::ADD;
        }
        else
        {
            ma.markers[i].action = visualization_msgs::Marker::DELETE;
        }
        //set pose of marker
        ma.markers[i].pose.position.x = x_obstacle;
        ma.markers[i].pose.position.y = y_obstacle;
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
        ma.markers[i].color.r = 0.0;
        ma.markers[i].color.g = 1.0;
        ma.markers[i].color.b = 0.0;
        ma.markers[i].color.a = 1.0;

        // ma.markers[i].lifetime = ros::Duration();


        // ma.markers.push_back(marker);
    }
    // ROS_ERROR_STREAM("MARKER: " << ma);
    marker_flag = true;

}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "slam");
    ros::NodeHandle nh_prv("~");
    ros::NodeHandle nh;

    nh_prv.param("rate", rate, 500);
    ros::Rate r(rate);

    if (!nh.hasParam("body_id"))
    {
        ROS_ERROR_STREAM("No param named 'body_id'.");
        return 1;
    }
    else
    {
        nh.getParam("body_id", body_id);
        // ROS_ERROR_STREAM("BODY ID " << body_id);
    }

    if (!nh.hasParam("wheel_left"))
    {
        ROS_ERROR_STREAM("No param named 'wheel_left'.");
        return 1;
    }
    else
    {
        nh.getParam("wheel_left", wheel_left);
        // ROS_ERROR_STREAM("WHEEL LEFT ID " << wheel_left);
    }

    if (!nh.hasParam("wheel_right"))
    {
        ROS_ERROR_STREAM("No param named 'wheel_right'.");
        return 1;
    }
    else
    {
        nh.getParam("wheel_right", wheel_right);
        // ROS_ERROR_STREAM("WHEEL RIGHT ID " << wheel_right);
    }

    nh.param<std::string>("odom_id", odom_id, "odom");

    nh.getParam("nusim/obstacles/radius", radius);
    nh.getParam("nusim/obstacles/height", height);
    nh.getParam("nusim/sensor/range_max", range_max);


    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("slam_markers", 1000);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("slam_path", 1000);

    ros::Subscriber joint_sub = nh.subscribe("joint_states", 1000, jointCallback);
    // ros::Subscriber fake_sensor_sub = nh.subscribe("fake_sensor", 1000, landmarkCallback);
    ros::Subscriber landmark_sub = nh.subscribe("landmarks", 1000, landmarkCallback);

    ros::ServiceServer set_pose = nh.advertiseService("set_pose", poseCallback);

    ros::Timer marker_timer = nh.createTimer(ros::Duration(0.2), markerCallback);

    tf2_ros::TransformBroadcaster br;
    
    geometry_msgs::TransformStamped world_blue_tf;
    geometry_msgs::TransformStamped odom_green_tf;
    geometry_msgs::TransformStamped map_odom_tf;

    tf2::Quaternion q;

    while(ros::ok())
    {
        geometry_msgs::PoseWithCovariance p;
        p.pose.position.x = x;
        p.pose.position.y = y;
        p.pose.position.z = 0.0;

        geometry_msgs::TwistWithCovariance t;
        t.twist.linear.x = u.x;
        t.twist.linear.y = u.y;
        t.twist.angular.z = u.ang;

        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = body_id;
        odom.child_frame_id = odom_id;
        odom.pose = p;
        odom.twist = t;
        odom_pub.publish(odom);

        world_blue_tf.header.stamp = ros::Time::now();

        world_blue_tf.header.frame_id = "world";
        world_blue_tf.child_frame_id = "blue_base_footprint";

        world_blue_tf.transform.translation.x = x;
        world_blue_tf.transform.translation.y = y;
        world_blue_tf.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta);

        world_blue_tf.transform.rotation.x = q.x();
        world_blue_tf.transform.rotation.y = q.y();
        world_blue_tf.transform.rotation.z = q.z();
        world_blue_tf.transform.rotation.w = q.w();
        br.sendTransform(world_blue_tf);

        // ROS_ERROR_STREAM("X: " << e.config().x);
        // ROS_ERROR_STREAM("Y: " << e.config().y);
        // ROS_ERROR_STREAM("ANG: " << e.config().ang);

        turtlelib::Vector2D xy{e.config().x, e.config().y};
        turtlelib::Transform2D Tmb(xy, e.config().ang);
        turtlelib::Transform2D Tob(turtlelib::Vector2D{x, y}, theta);

        turtlelib::Transform2D Tmo = Tmb*Tob.inv();

        // ROS_ERROR_STREAM("Tmb: " << Tmb);
        // ROS_ERROR_STREAM("Tob: " << Tob);

        // ROS_ERROR_STREAM("X: " << Tmo.translation().x);
        // ROS_ERROR_STREAM("Y: " << Tmo.translation().y);

        map_odom_tf.header.stamp = ros::Time::now();

        map_odom_tf.header.frame_id = "map";
        map_odom_tf.child_frame_id = "odom";

        map_odom_tf.transform.translation.x = Tmo.translation().x;
        map_odom_tf.transform.translation.y = Tmo.translation().y;
        map_odom_tf.transform.translation.z = 0.0;
        // map_odom_tf.transform.translation.x = 0;
        // map_odom_tf.transform.translation.y = 0;
        // map_odom_tf.transform.translation.z = 0.0;

        // map_odom_tf.transform.translation.x = 1;
        // map_odom_tf.transform.translation.y = 1;
        // map_odom_tf.transform.translation.z = 0.0;

        q.setRPY(0, 0, Tmo.rotation());
        // q.setRPY(0, 0, 0);

        map_odom_tf.transform.rotation.x = q.x();
        map_odom_tf.transform.rotation.y = q.y();
        map_odom_tf.transform.rotation.z = q.z();
        map_odom_tf.transform.rotation.w = q.w();

        br.sendTransform(map_odom_tf);

        odom_green_tf.header.stamp = ros::Time::now();

        odom_green_tf.header.frame_id = "odom";
        odom_green_tf.child_frame_id = "green_base_footprint";

        odom_green_tf.transform.translation.x = x;
        odom_green_tf.transform.translation.y = y;
        odom_green_tf.transform.translation.z = 0.0;

        q.setRPY(0, 0, theta);

        odom_green_tf.transform.rotation.x = q.x();
        odom_green_tf.transform.rotation.y = q.y();
        odom_green_tf.transform.rotation.z = q.z();
        odom_green_tf.transform.rotation.w = q.w();

        br.sendTransform(odom_green_tf);

        if (marker_flag)
        {
            marker_pub.publish(ma);
            marker_flag = false;
        }

        ps.header.stamp = ros::Time::now();
        ps.header.frame_id = "map";

        ps.pose.position.x = e.config().x;
        ps.pose.position.y = e.config().y;

        path.header.stamp = ros::Time::now();
        path.header.frame_id = "map";

        path.poses.push_back(ps);

        path_pub.publish(path);


        // ROS_ERROR_STREAM("THE END");
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}