#include "ros/ros.h"

int main(int argc, char * argv[])
{
    static int rate;
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh_prv("~");

    ros::Rate r(nh_prv.getParam("rate", rate));

    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
    
}