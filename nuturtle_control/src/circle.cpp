/// \file
/// \brief publishes odometry messages and transform

/// PARAMETERS:
///     frequency: publishing frequency
///     
/// PUBLISHES: 
///     vel_pub (geometry_msgs/Twist): publishes to cmd_vel
///
/// SERVICES:
///     control (nuturtle_control/Control): updates linear and angular velocities
///     reverse (std_srvs/Empty): reverses direction of robot
///     stop (std_srvs/Empty): stops motion of robot    

#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include "nuturtle_control/Control.h"

enum class State{CTRL, REV, STOP, STOPPED};
static State state = State::STOP;

static auto frequency = 0;

static auto lin_vel = 0;
static auto ang_vel = 0;
static auto radius = 0;

/// \brief callback for control service
/// 
/// Takes radius and velocity and updates angular and linear velocity of robot
bool controlCallback(nuturtle_control::Control::Request & request, nuturtle_control::Control::Response & response)
{
    state = State::CTRL;
    ang_vel = request.velocity;
    radius = request.radius;
    lin_vel = radius*ang_vel;
    return true;
}

/// \brief callback for reverse service
///
/// Flips velocities to reverse direction of motion
bool reverseCallback(std_srvs::Empty::Request & request, std_srvs::Empty::Response & response)
{
    state = State::REV;
    ang_vel = -ang_vel;
    lin_vel = -lin_vel;
    return true;
}

/// \brief callback for stop service
///
/// Sets velocities to zero to stop motion
bool stopCallback(std_srvs::Empty::Request & request, std_srvs::Empty::Response & response)
{
    state = State::STOP;
    ang_vel = 0;
    lin_vel = 0;
    return true;
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "circle");
    ros::NodeHandle nh_prv("~");
    ros::NodeHandle nh;

    nh_prv.param("frequency", frequency, 100);
    ros::Rate r(frequency);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    ros::ServiceServer control = nh.advertiseService("control", controlCallback);
    ros::ServiceServer reverse = nh.advertiseService("reverse", reverseCallback);
    ros::ServiceServer stop = nh.advertiseService("stop", stopCallback);


    while(ros::ok())
    {
        if (state == State::STOP)
        {
            geometry_msgs::Twist t;
            t.linear.x = 0;
            t.angular.z = 0;

            vel_pub.publish(t);
            state = State::STOPPED;
        }
        
        else if (state != State::STOPPED)
        {
            geometry_msgs::Twist t;
            t.linear.x = lin_vel;
            t.angular.z = ang_vel;

            vel_pub.publish(t);
        }
        
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

