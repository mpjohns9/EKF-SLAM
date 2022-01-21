#include "ros/ros.h"
#include <std_msgs/UInt64.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/JointState.h>

static int timestep;

bool reset_callback(std_srvs::TriggerRequest & request, std_srvs::TriggerResponse & response)
{
    timestep = 0;
    return true;
}

int main(int argc, char * argv[])
{
    static int rate;
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh_prv("~");
    ros::NodeHandle nh;
    ros::Rate r(nh_prv.getParam("rate", rate));

    ros::Publisher pub_step = nh_prv.advertise<std_msgs::UInt64>("timestep", rate);
    ros::Publisher pub_joints = nh.advertise<sensor_msgs::JointState>("red/joint_states", rate);
    ros::ServiceServer reset = nh_prv.advertiseService("reset", reset_callback);
    
    timestep = 0;
    while(ros::ok())
    {
        std_msgs::UInt64 step;
        step.data = timestep;
        pub_step.publish(step);
        timestep++;
        ros::spinOnce();
        r.sleep();
    }
    return 0;
    
}