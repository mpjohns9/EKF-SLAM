/// \file
/// \brief Detects landmarks and publishes their location

/// PARAMETERS:
///
/// PUBLISHES: 
///
/// SUBSCRIBES:
///     laser_sub (sensor_msgs/LaserScan): gets laser scan data 
///
/// SERVICES: 


#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
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
#include <vector>

static auto angle_inc = 0.0;
static auto range_max = 0.0;
visualization_msgs::MarkerArray ma;

bool test_pub_flag = false;

void laserCallback(const sensor_msgs::LaserScan & msg)
{
    std::vector<std::vector<std::pair<double, double>>> cluster_list;
    std::vector<std::pair<double, double>> cluster;

    static auto threshold = 0.1;
    for (int i=0; i<int(msg.ranges.size()); i++)
    {
        auto angle = i*angle_inc;
        ROS_ERROR_STREAM("ANGLE: " << angle);

        // handle out of range case
        if (int(msg.ranges.at(i)) > range_max)
        {
            if (int(cluster.size()) < 3)
            {
                cluster.clear();
            }
            else
            {
                ROS_ERROR_STREAM("CLUSTER ADDED (OOR).");
                ROS_ERROR_STREAM("Length of Cluster List: " << int(cluster_list.size()));
                cluster_list.push_back(cluster);
                cluster.clear();
            }
            break;
        }

        // cluster range data
        if (int(cluster.size() > 0))
        {
            auto prev = std::get<1>(cluster.back());
            auto curr = msg.ranges.at(i);

            ROS_ERROR_STREAM("PREV: " << prev);
            ROS_ERROR_STREAM("CURR: " << curr);
            ROS_ERROR_STREAM("DIFF: " << prev-curr);


            if (abs(prev-curr) <= threshold)
            {
                cluster.push_back(std::pair<double, double>(angle, curr));
            }
            else
            {
                if (int(cluster.size()) >= 3)
                {
                    ROS_ERROR_STREAM("CLUSTER ADDED (NORM).");
                    ROS_ERROR_STREAM("Length of Cluster List: " << int(cluster_list.size()));
                    cluster_list.push_back(cluster);
                    cluster.clear();
                }
                else
                {
                    cluster.clear();
                    cluster.push_back(std::pair<double, double>(angle, curr));
                }
            }
        }
        else
        {   
            auto curr = msg.ranges.at(i);
            cluster.push_back(std::pair<double, double>(angle, curr));
        }
    }

    // // check for same cluster at beginning/end of range vec
    // if (int(cluster_list.size()) > 1)
    // {
    //     std::vector<std::pair<double, double>> first = cluster_list.front();
    //     std::vector<std::pair<double, double>> last = cluster_list.back();

    //     if (abs(std::get<1>(first.front())-std::get<1>(last.back())) <= threshold)
    //     {
    //         std::vector<std::pair<double, double>> combined;
    //         for (int i=0; i<int(last.size()); i++)
    //         {
    //             first.push_back(last.at(i));
    //         }
    //         // first.insert(first.end(), last.begin(), last.end());
    //         cluster_list.at(0) = combined;
    //         cluster_list.pop_back();
    //     }
    // }

    int size = 0;
    for (int i=0; i<int(cluster_list.size()); i++)
    {
        size += int(cluster_list.at(i).size());
    }
    

    // print clusters for testing
    ma.markers.resize(size);
    test_pub_flag = false;
    int counter = 0;
    for (int i=0; i<int(cluster_list.size()); i++)
    {
        // ROS_ERROR_STREAM_ONCE("CLUSTER: -------------------------------------");
        for (int j=0; j<int(cluster_list.at(i).size()); j++)
        {
            // ROS_ERROR_STREAM_ONCE(std::get<1>(cluster_list.at(i).at(j)));
            auto angle = std::get<0>(cluster_list.at(i).at(j));
            auto r = std::get<1>(cluster_list.at(i).at(j));

            auto x = r*cos(angle);
            auto y = r*sin(angle);

            //set header and timestamp
            ma.markers[counter].header.frame_id = "red_base_scan";
            ma.markers[counter].header.stamp = ros::Time::now();

            //set id diff for each marker
            ma.markers[counter].id = counter;

            //set color and action
            ma.markers[counter].type = visualization_msgs::Marker::SPHERE;
            // ROS_ERROR_STREAM("r (Obstacle " << i << "): " << r);
            
            //set pose of marker
            ma.markers[counter].pose.position.x = x;
            ma.markers[counter].pose.position.y = y;
            ma.markers[counter].pose.position.z = 0;
            ma.markers[counter].pose.orientation.x = 0;
            ma.markers[counter].pose.orientation.y = 0;
            ma.markers[counter].pose.orientation.z = 0;
            ma.markers[counter].pose.orientation.w = 1;

            //set size
            ma.markers[counter].scale.x = 0.1;
            ma.markers[counter].scale.y = 0.1;
            ma.markers[counter].scale.z = 0.1;


            //set color
            ma.markers[counter].color.r = 0.0;
            ma.markers[counter].color.g = 1.0;
            ma.markers[counter].color.b = 0.0;
            ma.markers[counter].color.a = 1.0;
            counter++;

        }
    }
    test_pub_flag = true;

    // ROS_ERROR_STREAM_ONCE("____________________________________");

    // // for each distance, calculate location of landmark relative to robot
    // for (int i=0; i<int(cluster_list.size()); i++)
    // {
        
    // } 


}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "landmarks");
    ros::NodeHandle nh;
    
    ros::Rate r(500);

    nh.getParam("nusim/sensor/angle_increment", angle_inc);
    nh.getParam("nusim/sensor/range_max", range_max);

    ros::Publisher test_cluster_pub = nh.advertise<visualization_msgs::MarkerArray>("test_cluster", 1000);

    ros::Subscriber laser_sub = nh.subscribe("laser_scan", 1000, laserCallback);
    
    while(ros::ok())
    {
        if (test_pub_flag)
        {
            test_cluster_pub.publish(ma);
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
    
}