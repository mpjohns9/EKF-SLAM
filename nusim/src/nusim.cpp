#include "ros/ros.h"
#include <std_msgs/UInt64.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

static int timestep;

bool reset_callback(std_srvs::TriggerRequest & request, std_srvs::TriggerResponse & response)
{
    timestep = 0;
    return true;
}

int main(int argc, char * argv[])
{
    static int rate;
    double x_0, y_0, theta_0;

    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh_prv("~");
    ros::NodeHandle nh;
    ros::Rate r(nh_prv.param("rate", rate, 500));

    nh_prv.param("x0", x_0, 0.0);
    nh_prv.param("y0", y_0, 0.0);
    nh_prv.param("theta0", theta_0, 0.0);

    std_msgs::UInt64 step;

    ros::Publisher pub_step = nh_prv.advertise<std_msgs::UInt64>("timestep", rate);
    ros::Publisher pub_joints = nh.advertise<sensor_msgs::JointState>("red/joint_states", rate);

    sensor_msgs::JointState js;

    ros::ServiceServer reset = nh_prv.advertiseService("reset", reset_callback);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transform;
    tf2::Quaternion q;
    
    timestep = 0;
    while(ros::ok())
    {
        transform.header.stamp = ros::Time::now();

        transform.header.frame_id = "world";
        transform.child_frame_id = "red_base_footprint";

        transform.transform.translation.x = x_0;
        transform.transform.translation.y = y_0;
        transform.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_0);

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