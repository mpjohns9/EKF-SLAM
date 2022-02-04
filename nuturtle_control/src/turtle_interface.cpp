/// \file
/// \brief enables control of the turtlebot

/// PARAMETERS: 
///     ticks_to_rad: radians per encoder tick
///
/// SUBSCRIBES: 
///     vel_sub (geometry_msgs/Twist): subscribes to cmd_vel
///     sensor_sub (nuturtlebot_msgs/SensorData): subscribes to sensor_data
///
/// PUBLISHES:
///     wheel_pub (nuturtlebot_msgs/WheelCommands): publishes wheel_cmd
///     joint_pub (sensor_msgs/JointState): publishes joint_states

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"

// initialize classes
turtlelib::diffDrive dd;
turtlelib::Transform2D tf;

// publishing rate 
static auto rate = 0;

// robot configuration
static auto x = 0;
static auto y = 0;
static auto theta = 0;

// wheel velocities
static auto lwheel_vel = 0;
static auto rwheel_vel = 0;

// wheel positions
static auto lwheel_pos = 0;
static auto rwheel_pos = 0;

// initialize ticks to radians conversion
static auto ticks_to_rad = 0;

/// \brief callback for the cmd_vel subscriber
/// \param msg - geometry_msgs/Twist message obj
/// Uses the diffDrive library to convert twist to wheel velocities
void velCallback(const geometry_msgs::Twist & msg)
{
    tf::Twist2D V{msg.angular.z, msg.linear.x, msg.linear.y};
    dd::WheelVel vel = dd::inv_kin(V);
    lwheel_vel = vel.l_wheel;
    rwheel_vel = vel.r_wheel;
}

/// \brief callback for the sensor_data subscriber
/// \param msg - nuturtlebot_msgs/SensorData message obj
/// Sets wheel positions (in encoder ticks)
void sensorCallback(const nuturtlebot_msgs::SensorData & msg)
{
    lwheel_pos = msg.left_encoder;
    rwheel_pos = msg.right_encoder;
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle nh_prv("~");
    ros::NodeHandle nh;
    ros::Rate r(nh_prv.param("rate", rate, 500));

    // get encoder ticks conversion param
    // if it doesn't exist, throw an error
    if (!nh.hasParam("encoder_ticks_to_rad"))
    {
        ROS_ERROR_STREAM("No param named 'encoder_ticks_to_rad'.");
        return 1;
    }
    else
    {
        nh.getParam("encoder_ticks_to_rad", ticks_to_rad);
    }

    ros::Publisher wheel_pub = nh.advertise<nuturtlebot_msgs::WheelCommands>("wheel_cmd", 1000);
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);

    ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 1000, velCallback);
    ros::Subscriber sensor_sub = nh.subscribe("sensor_data", 1000, sensorCallback);


    while(ros::ok())
    {
        nuturtlebot_msgs::WheelCommands cmd;
        cmd.left_velocity = lwheel_vel;
        cmd.right_velocity = rwheel_vel;
        wheel_pub.publish(cmd);

        sensor_msgs::JointState js;
        js.name = ["wheel_left_joint", "wheel_right_joint"];
        js.position = [lwheel_pos*ticks_to_rad, rwheel_pos*ticks_to_rad];
        js.velocity = [lwheel_vel, rwheel_vel];
        joint_pub.publish(js);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}