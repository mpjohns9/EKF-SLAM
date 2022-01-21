#include "ros/ros.h"
#include <std_msgs/UInt64.h>

int main(int argc, char * argv[])
{
    static int rate;
    int timestep = 0;
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh_prv("~");
    ros::Rate r(nh_prv.getParam("rate", rate));

    ros::Publisher pub = nh_prv.advertise<std_msgs::UInt64>("timestep", rate);
    

    while(ros::ok())
    {
        std_msgs::UInt64 step;
        step.data = timestep;
        pub.publish(step);
        timestep++;
        ros::spinOnce();
        r.sleep();
    }
    return 0;
    
}