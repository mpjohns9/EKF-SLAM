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
#include "turtlelib/circle_fit.hpp"
#include <vector>

static auto angle_inc = 0.0;
static auto range_max = 0.0;
static auto range_min = 0.0;
visualization_msgs::MarkerArray ma;
visualization_msgs::MarkerArray circle_ma;

bool landmark_flag = false;
bool test_pub_flag = false;

void laserCallback(const sensor_msgs::LaserScan & msg)
{
    std::vector<std::vector<std::pair<double, double>>> cluster_list;
    std::vector<std::pair<double, double>> cluster;

    static auto threshold = 0.1;
    for (int i=0; i<int(msg.ranges.size()); i++)
    {
        // ROS_ERROR_STREAM("LENGTH OF MSG: " << msg.ranges.size());
        auto angle = i*angle_inc;
        // ROS_ERROR_STREAM("ANGLE: " << angle);

        // handle out of range case
        if (int(msg.ranges.at(i)) > range_max)
        {
            if (int(cluster.size()) < 4)
            {
                cluster.clear();
            }
            else
            {
                // ROS_ERROR_STREAM("CLUSTER ADDED (OOR).");
                cluster_list.push_back(cluster);
                cluster.clear();
                // ROS_ERROR_STREAM("Length of Cluster List: " << int(cluster_list.size()));
            }
            continue;
        }

        // cluster range data
        if (int(cluster.size() > 0))
        {
            auto prev = std::get<1>(cluster.back());
            auto curr = msg.ranges.at(i);

            // ROS_ERROR_STREAM("PREV: " << prev);
            // ROS_ERROR_STREAM("CURR: " << curr);
            // ROS_ERROR_STREAM("DIFF: " << prev-curr);


            if (abs(prev-curr) <= threshold)
            {
                cluster.push_back(std::pair<double, double>(angle, curr));
            }
            else
            {
                if (int(cluster.size()) >= 4)
                {
                    // ROS_ERROR_STREAM("CLUSTER ADDED (NORM).");
                    // ROS_ERROR_STREAM("Length of Cluster List: " << int(cluster_list.size()));
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

    ma.markers.resize(size);
    test_pub_flag = false;
    int counter = 0;
    // ROS_ERROR_STREAM("CLUSTER LIST SIZE: " << cluster_list.size());
    std::vector<std::tuple<double, double, double>> circle_mark;
    for (int i=0; i<int(cluster_list.size()); i++)
    {
        // ROS_ERROR_STREAM("CLUSTER " << i);
        std::vector<double> cluster_x;
        std::vector<double> cluster_y;
        double color = 0.05 + (i*0.95/int(cluster_list.size()));
        // ROS_ERROR_STREAM("CLUSTER " << i << "_______________________________________");
        // ROS_ERROR_STREAM("CLUSTER SIZE " << cluster_list.at(i).size());
        for (int j=0; j<int(cluster_list.at(i).size()); j++)
        {
            // ROS_ERROR_STREAM(std::get<1>(cluster_list.at(i).at(j)));
            auto angle = std::get<0>(cluster_list.at(i).at(j));
            auto r = std::get<1>(cluster_list.at(i).at(j));

            auto x = r*cos(angle);
            auto y = r*sin(angle);

            // ROS_ERROR_STREAM("ANGLE: " << angle);
            // ROS_ERROR_STREAM("X: " << x);
            // ROS_ERROR_STREAM("Y: " << y);

            cluster_x.push_back(x);
            cluster_y.push_back(y);

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
            ma.markers[counter].color.g = color;
            ma.markers[counter].color.b = 0.0;
            ma.markers[counter].color.a = 1.0;

            counter++;

        }

        // ROS_ERROR_STREAM("TOTAL CLUSTERS: " << cluster_list.size());

        turtlelib::circleFit cf(cluster_x, cluster_y);
        double c_x, c_y, r;
        std::tie(c_x, c_y, r) = cf.fit_circle();

        // ROS_ERROR_STREAM("FIT PARAMS: " << c_x << c_y << r);

        // ROS_ERROR_STREAM("DONE");

        if (cf.classify_circle())
        {
            // ROS_ERROR_STREAM("CLASSIFIED CIRCLE");
            circle_mark.push_back(std::make_tuple(c_x, c_y, r));
        }
        // ROS_ERROR_STREAM("DONE");

    }

    circle_ma.markers.resize(circle_mark.size());
    // ROS_ERROR_STREAM("CIRCLE MARK SIZE: " << circle_mark.size());
    for (int i=0; i<int(circle_mark.size()); i++)
    {
        //set header and timestamp
        circle_ma.markers[i].header.frame_id = "red_base_scan";
        circle_ma.markers[i].header.stamp = ros::Time::now();

        //set id diff for each marker
        circle_ma.markers[i].id = i;

        //set color and action
        circle_ma.markers[i].type = visualization_msgs::Marker::CYLINDER;
        // ROS_ERROR_STREAM("r (Obstacle " << i << "): " << r);

        // if (abs(c_x) > 3 or abs(c_y) > 3)
        // {
        //     circle_ma.markers[i].action = visualization_msgs::Marker::DELETE;
        // }
        
        //set pose of marker
        circle_ma.markers[i].pose.position.x = std::get<0>(circle_mark.at(i));
        circle_ma.markers[i].pose.position.y = std::get<1>(circle_mark.at(i));
        circle_ma.markers[i].pose.position.z = 0;
        circle_ma.markers[i].pose.orientation.x = 0;
        circle_ma.markers[i].pose.orientation.y = 0;
        circle_ma.markers[i].pose.orientation.z = 0;
        circle_ma.markers[i].pose.orientation.w = 1;

        //set size
        circle_ma.markers[i].scale.x = 2*std::get<2>(circle_mark.at(i));
        circle_ma.markers[i].scale.y = 2*std::get<2>(circle_mark.at(i));
        circle_ma.markers[i].scale.z = 0.25;


        //set color
        circle_ma.markers[i].color.r = 1.0;
        circle_ma.markers[i].color.g = 1.0;
        circle_ma.markers[i].color.b = 0.0;
        circle_ma.markers[i].color.a = 1.0;
    }
    // ROS_ERROR_STREAM_ONCE(circle_ma);
    test_pub_flag = true;

    // ROS_ERROR_STREAM_ONCE("____________________________________");

    // // for each distance, calculate location of landmark relative to robot
    // for (int i=0; i<int(cluster_list.size()); i++)
    // {
        
    // } 


}

void landmarkCallback(const ros::TimerEvent&)
{
    landmark_flag = true;   
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "landmarks");
    ros::NodeHandle nh;
    
    ros::Rate r(500);

    nh.getParam("nusim/sensor/angle_increment", angle_inc);
    nh.getParam("nusim/sensor/range_max", range_max);
    nh.getParam("nusim/sensor/range_min", range_min);

    ros::Publisher test_cluster_pub = nh.advertise<visualization_msgs::MarkerArray>("test_cluster", 1000);
    ros::Publisher landmark_pub = nh.advertise<visualization_msgs::MarkerArray>("landmarks", 1000);


    ros::Subscriber laser_sub = nh.subscribe("laser_scan", 1000, laserCallback);

    ros::Timer landmark_timer = nh.createTimer(ros::Duration(0.2), landmarkCallback);

    
    while(ros::ok())
    {
        // ROS_ERROR_STREAM("LANDMARKS");
        if (test_pub_flag)
        {
            // ROS_ERROR_STREAM("PUBLISHING");
            test_cluster_pub.publish(ma);
        }

        if (landmark_flag)
        {
            landmark_pub.publish(circle_ma);
            landmark_flag = false;
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
    
}