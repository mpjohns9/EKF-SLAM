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
///     pub_step (std_msgs/UInt64): publishes timestep of simulation
///     marker_pub (visualization_msgs/MarkerArray): publishes array of cylindrical markers
///     wall_pub (visualization_msgs/MarkerArray): publishes array of cubes (walls)
///     sensor_pub (nuturtlebot_msgs/SensorData): publishes updated wheel positions
///     path_pub (nav_msgs/Path): publishes path of robot
/// SUBSCRIBES:
///     wheel_sub (nuturtlebot_msgs/WheelCommands): updates wheel velocities
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
#include <geometry_msgs/PoseStamped.h>
#include "nusim/Teleport.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/console.h>
#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include "turtlelib/diff_drive.hpp"
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <random>
#include <cmath>
#include <vector>

turtlelib::diffDrive dd;

static auto timestep = 0;
static auto rate = 0;

// initial position and orientation
static auto x_0 = 0.0, y_0 = 0.0, theta_0 = 0.0;

// current position and orientation
static auto x = 0.0, y = 0.0, theta = 0.0;

// obstacle x/y coord arrays
static std::vector<double> obs_x;
static std::vector<double> obs_y;

// height and radius of cylindrical objects
static double radius = 0.0;
static double height = 0.0;

// left and right wheel positions
static auto lwheel_pos = 0.0;
static auto rwheel_pos = 0.0;

// left and right wheel velocities
static auto lwheel_vel = 0.0;
static auto rwheel_vel = 0.0;

// x, y length and thickness of arena (walls)
static auto x_length = 0.0;
static auto y_length = 0.0;
static auto w_thick = 0.1;

// initialize conversions
static auto ticks_to_rad = 0.0;
static auto cmd_to_radsec = 0.0;

// initialize noise params
static auto vnoise_mean = 0.0;
static auto vnoise_stddev = 0.0;
static auto slip_min = 0.0;
static auto slip_max = 0.0;

// initialize sensor params
static auto angle_min = 0.0;
static auto angle_max = 0.0;
static auto angle_inc = 0.0;
static auto time_inc = 0.0;
static auto scan_time = 0.0;
static auto range_min = 0.0;
static auto range_max = 0.0;
static auto num_samples = 0.0;

static auto basic_sensor_variance = 0.0;

static auto collision_rad = 0.0;

visualization_msgs::MarkerArray fake_ma;
sensor_msgs::LaserScan laser;
static auto fake_sensor_flag = false;
static auto scan_flag = false;
static auto collision_flag = false;

/// \brief RNG seeding function
///
/// ensures random number generator is only seeded once
//  source: https://nu-msr.github.io/navigation_site/lectures/gaussian.html
std::mt19937 & get_random()
 {
     // static variables inside a function are created once and persist for the remainder of the program
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     // we return a reference to the pseudo-random number genrator object. This is always the
     // same object every time get_random is called
     return mt;
 }

std::normal_distribution<double> on(0.0, basic_sensor_variance);
double obs_noise = on(get_random());

/// \brief callback for reset service
///
/// resets timestep to 0 and robot to initial position
bool resetCallback(std_srvs::TriggerRequest & request, std_srvs::TriggerResponse & response)
{
    timestep = 0;
    x = 0.0;
    y = 0.0;
    theta = 0.0;

    dd = turtlelib::diffDrive();
    return true;
}


/// \brief callback for teleport service
///
/// teleports robot to x, y, theta position
bool teleportCallback(nusim::Teleport::Request & request, nusim::Teleport::Response & response)
{
    x = request.x;
    y = request.y;
    theta = request.theta;

    turtlelib::Config c = {theta, x, y};
    dd = turtlelib::diffDrive(c);
    return true;
}

/// \brief callback for wheel_cmd subscriber
/// \param msg - nuturtlebot_msgs/WheelCommands message obj
/// receives motion commands for the turtlebot
void wheelCallback(const nuturtlebot_msgs::WheelCommands & msg)
{   
    double vel_noise = 0.0;
    double pos_noise = 1.0;
    
    if ((msg.left_velocity != 0) or (msg.right_velocity != 0)) {
        // To generate a gaussian variable:
        std::normal_distribution<double> vn(vnoise_mean, vnoise_stddev);
        vel_noise = vn(get_random());

        std::uniform_real_distribution<double> pn(slip_min, slip_max);
        pos_noise = pn(get_random());
    }

    lwheel_vel = msg.left_velocity*cmd_to_radsec/rate;
    rwheel_vel = msg.right_velocity*cmd_to_radsec/rate;

    double lvel_noisy = lwheel_vel + vel_noise;
    double rvel_noisy = rwheel_vel + vel_noise;


    lwheel_pos += lvel_noisy;
    rwheel_pos += rvel_noisy;

    turtlelib::WheelPos pos {lwheel_pos, rwheel_pos};
    turtlelib::Config c;

    c = std::get<0>(dd.fwd_kin(pos));

    double new_x = c.x;
    double new_y = c.y;
    double new_theta = c.ang;

    for (int i=0;i<int(obs_x.size());i++)
    {
        double x_obstacle = obs_x[i] + obs_noise;
        double y_obstacle = obs_y[i] + obs_noise;

        double distance = std::sqrt(std::pow(x_obstacle - new_x, 2) + std::pow(y_obstacle - new_y, 2));
        double tan_dist = (radius + collision_rad);

        // ROS_ERROR_STREAM("DISTANCE: " << distance);
        // ROS_ERROR_STREAM("TAN DISTANCE: " << tan_dist);
        // ROS_ERROR_STREAM("_______________");

        if (distance < tan_dist and !collision_flag)
        {
            // ROS_ERROR_STREAM("----------------COLLISION DETECTED--------------------");
            c.x = x;
            c.y = y;
            c.ang = theta;
            collision_flag = true;
            break;
        //     double t = (tan_dist-distance)/distance;
        //     x = ((1 - t)*x_obstacle) + t*x;
        //     y = ((1 - t)*y_obstacle) + t*y;
        }
        collision_flag = false;
    }

    if (!collision_flag)
    {
        x = new_x;
        y = new_y;
        theta = new_theta;
    }

    turtlelib::WheelVel v;
    v.l_vel = lvel_noisy*cmd_to_radsec;
    v.r_vel = rvel_noisy*cmd_to_radsec;

    lwheel_pos += (pos_noise*lvel_noisy);
    rwheel_pos += (pos_noise*rvel_noisy);

    pos = {lwheel_pos, rwheel_pos};

    dd = turtlelib::diffDrive(c, pos, v);
    
}

void marktimerCallback(const ros::TimerEvent&)
{
    std::vector<double> obs_vec;
    fake_ma.markers.resize(obs_x.size());
    for (int i=0;i<int(obs_x.size());i++)
    {
        turtlelib::Vector2D vec{x, y};
        turtlelib::Transform2D Twb(vec, theta);
        turtlelib::Transform2D Tbw = Twb.inv();

        double x_obstacle = obs_x[i];
        double y_obstacle = obs_y[i];

        turtlelib::Vector2D obs_vec {x_obstacle, y_obstacle};
        turtlelib::Vector2D obs_vec_b = Tbw(obs_vec);

        double obs_theta = atan2(obs_vec_b.x, obs_vec_b.y) + obs_noise;
        double r = std::sqrt(std::pow(obs_vec_b.x, 2) + std::pow(obs_vec_b.y, 2));

        // double distance = std::sqrt(std::pow(x_obstacle - x, 2) + std::pow(y_obstacle - y, 2));
        // visualization_msgs::Marker marker;

        //set header and timestamp
        fake_ma.markers[i].header.frame_id = "world";
        fake_ma.markers[i].header.stamp = ros::Time::now();

        //set id diff for each marker
        fake_ma.markers[i].id = i;

        //set color and action
        fake_ma.markers[i].type = visualization_msgs::Marker::CYLINDER;
        // ROS_ERROR_STREAM("r (Obstacle " << i << "): " << r);
        if (abs(r) <= range_max)
        {
            fake_ma.markers[i].action = visualization_msgs::Marker::ADD;
        }
        else
        {
            fake_ma.markers[i].action = visualization_msgs::Marker::DELETE;
        }
        //set pose of marker
        fake_ma.markers[i].pose.position.x = obs_x.at(i);
        fake_ma.markers[i].pose.position.y = obs_y.at(i);
        fake_ma.markers[i].pose.position.z = 0;
        fake_ma.markers[i].pose.orientation.x = 0;
        fake_ma.markers[i].pose.orientation.y = 0;
        fake_ma.markers[i].pose.orientation.z = 0;
        fake_ma.markers[i].pose.orientation.w = 1;

        //set size
        fake_ma.markers[i].scale.x = 2*radius;
        fake_ma.markers[i].scale.y = 2*radius;
        fake_ma.markers[i].scale.z = height;


        //set color
        fake_ma.markers[i].color.r = 0.0;
        fake_ma.markers[i].color.g = 1.0;
        fake_ma.markers[i].color.b = 0.0;
        fake_ma.markers[i].color.a = 1.0;

        // ma.markers[i].lifetime = ros::Duration();


        // ma.markers.push_back(marker);
    }
    fake_sensor_flag = true;

}

void scantimerCallback (const ros::TimerEvent&)
{
    laser.header.stamp = ros::Time::now();
    laser.header.frame_id = "red_base_scan";

    laser.angle_min = angle_min;
    laser.angle_max = angle_max;
    laser.angle_increment = angle_inc;
    laser.time_increment = time_inc;
    laser.scan_time = scan_time;
    laser.range_min = range_min;
    laser.range_max = range_max;

    laser.ranges.resize(num_samples);
    double ang = 0.0;
    for (int j=0;j<num_samples;j++)
    {   
        ROS_ERROR_STREAM("ANGLE " << ang);
        std::vector<double> int_vec;
        for (int i=0;i<int(obs_x.size());i++)
        {
            // ROS_ERROR_STREAM("OBSTACLE " << i);
            double r_min = range_min;
            double xb1 = r_min*cos(ang);
            double yb1 = r_min*sin(ang);

            double r_max = range_max;
            double xb2 = r_max*cos(ang);
            double yb2 = r_max*sin(ang);

            turtlelib::Vector2D obs_vec{obs_x[i], obs_y[i]};
            turtlelib::Vector2D b1_vec{xb1, yb1};
            turtlelib::Vector2D b2_vec{xb2, yb2};

            turtlelib::Vector2D v{x, y};

            turtlelib::Transform2D Twb(v, theta);
            turtlelib::Transform2D Two(obs_vec);
            turtlelib::Transform2D Tob = (Two.inv())*Twb;

            turtlelib::Vector2D o1_vec = Tob(b1_vec);
            turtlelib::Vector2D o2_vec = Tob(b2_vec);

            // ROS_ERROR_STREAM("VEC 1: " << o1_vec);
            // ROS_ERROR_STREAM("VEC 2: " << o2_vec);

            double dx = o2_vec.x - o1_vec.  x;
            double dy = o2_vec.y - o1_vec.y;
            // ROS_ERROR_STREAM("DX: " << dx);
            // ROS_ERROR_STREAM("DY: " << dy);
            double dr = sqrt(pow(dx, 2) + pow(dy, 2));

            double D = (o1_vec.x*o2_vec.y) - (o2_vec.x*o1_vec.y);
            // ROS_ERROR_STREAM("D: " << D);

            double disc = (pow(collision_rad, 2)*pow(dr, 2)) - pow(D, 2);
            // ROS_ERROR_STREAM("DISC " << disc);

            if (turtlelib::almost_equal(disc, 0.0) or disc > 0)
            {
                auto sgn = 0;
                if (dy < 0.0)
                {
                    sgn = -1;
                }
                else
                {
                    sgn = 1;
                }
                
                double x_int1 = ((D*dy) + (sgn*dx*sqrt(disc)))/pow(dr, 2);
                double y_int1 = ((-D*dx) + (abs(dy)*sqrt(disc)))/pow(dr, 2);
                turtlelib::Vector2D int1_vec{x_int1, y_int1};
                turtlelib::Vector2D int1_vec_b = Tob.inv()(int1_vec);
    
                // ROS_ERROR_STREAM("_____________________________________");
                // ROS_ERROR_STREAM("DX: " << dx);
                // ROS_ERROR_STREAM("DY: " << dy);
                // ROS_ERROR_STREAM("_____________________________________");

                double dist1 = sqrt(pow(int1_vec_b.x, 2) + pow(int1_vec_b.y, 2));
                double ref_dist = sqrt(pow(int1_vec_b.x - b1_vec.x, 2) + pow(int1_vec_b.y - b1_vec.y, 2));

                // ROS_ERROR_STREAM("DIST1: " << dist1);
                // ROS_ERROR_STREAM("REF_DIST1: " << ref_dist);


                if (ref_dist < dist1)
                {
                    int_vec.push_back(dist1);
                }
                // ROS_ERROR_STREAM("D1: " << dist1);


                double x_int2 = ((D*dy) - (sgn*dx*sqrt(disc)))/pow(dr, 2);
                double y_int2 = ((-D*dx) - (abs(dy)*sqrt(disc)))/pow(dr, 2);
                turtlelib::Vector2D int2_vec{x_int2, y_int2};
                turtlelib::Vector2D int2_vec_b = Tob.inv()(int2_vec);

                double dist2 = sqrt(pow(int2_vec_b.x, 2) + pow(int2_vec_b.y, 2));
                ref_dist = sqrt(pow(int2_vec_b.x - b1_vec.x, 2) + pow(int2_vec_b.y - b1_vec.y, 2));

                // ROS_ERROR_STREAM("DIST2: " << dist1);
                // ROS_ERROR_STREAM("REF_DIST2: " << ref_dist);

                if (ref_dist < dist2)
                {
                    int_vec.push_back(dist2);
                }
                // ROS_ERROR_STREAM("D2: " << dist2);

            }

        }

        double x_wall = (x_length/2) - (w_thick/2);
        double y_wall = (y_length/2) - (w_thick/2);
        std::vector<double> x_wall_vec {x_wall, x_wall, -x_wall, -x_wall};
        std::vector<double> y_wall_vec {y_wall, -y_wall, -y_wall, y_wall};
        // std::vector<double> x_wall_vec {-x_wall, x_wall, x_wall};
        // std::vector<double> y_wall_vec {y_wall, y_wall, -y_wall};
        for (int i =0;i<4;i++)
        {
            double x_w1 = 0.0;
            double x_w2 = 0.0;
            double y_w1 = 0.0;
            double y_w2 = 0.0;

            // ROS_ERROR_STREAM("WALL " << i);
            if (i == 3)
            {
                x_w1 = x_wall_vec.at(i);
                y_w1 = y_wall_vec.at(i);

                x_w2 = x_wall_vec.at(0);
                y_w2 = y_wall_vec.at(0);
            }
            else
            {
                x_w1 = x_wall_vec.at(i);
                y_w1 = y_wall_vec.at(i);

                x_w2 = x_wall_vec.at(i+1);
                y_w2 = y_wall_vec.at(i+1);

            }
           
            double x_robot2 = x + range_max*cos(theta + ang);
            double y_robot2 = y + range_max*sin(theta + ang);

            double dnm = ((x_w1 - x_w2)*(y - y_robot2)) - ((y_w1 - y_w2)*(x - x_robot2));

            if (!turtlelib::almost_equal(dnm, 0.0))
            {
                

                double line_int_x = (((x_w1*y_w2) - (y_w1*x_w2))*(x - x_robot2) - ((x_w1 - x_w2)*(x*y_robot2 - y*x_robot2)))/dnm;

                double line_int_y = (((x_w1*y_w2) - (y_w1*x_w2))*(y - y_robot2) - ((y_w1 - y_w2)*(x*y_robot2 - y*x_robot2)))/dnm;
                
                double dist = sqrt(pow(x - line_int_x, 2) + pow(y - line_int_y, 2));

                ROS_ERROR_STREAM("_____________________________________");

                ROS_ERROR_STREAM("DENOM: " << dnm);
                ROS_ERROR_STREAM("DIST: " << dist);


                ROS_ERROR_STREAM("WALL " << i);
                ROS_ERROR_STREAM("XINTWALL: " << line_int_x);
                ROS_ERROR_STREAM("YINTWALL: " << line_int_y);

                ROS_ERROR_STREAM("X ROBOT 1: " << x);
                ROS_ERROR_STREAM("Y ROBOT 1: " << y);
                ROS_ERROR_STREAM("X ROBOT 2: " << x_robot2);
                ROS_ERROR_STREAM("Y ROBOT 2: " << y_robot2);

                ROS_ERROR_STREAM("X WALL 1: " << x_w1);
                ROS_ERROR_STREAM("Y WALL 1: " << y_w1);
                ROS_ERROR_STREAM("X WALL 2: " << x_w2);
                ROS_ERROR_STREAM("Y WALL 2: " << y_w2);

                ROS_ERROR_STREAM("_____________________________________");

                double ref_dist = sqrt(pow(x_robot2 - line_int_x, 2) + pow(y_robot2 - line_int_y, 2));
                if (ref_dist < dist)
                {
                    int_vec.push_back(dist);
                }
                // ROS_ERROR_STREAM("DWALL: " << dist);
            }

            // double dx_wall = x_w2 - x_w1;
            // double dy_wall = y_w2 - y_w1;
            
            // double dx_robot = range_max*cos(ang);
            // double dy_robot = range_max*sin(ang);

            // double m_wall = dy_wall/dx_wall;
            // double m_robot = dy_robot/dx_robot;

            // double c_wall = y_w1 - (m_wall*x_w1);
            // double c_robot = y - (m_robot*x);

            // if (!turtlelib::almost_equal(m_wall, m_robot))
            // {
            //     double x_int = (c_wall - c_robot)/(m_robot - m_wall);
            //     double y_int = (m_robot*x_int) + c_robot;
            //     double dist = std::sqrt(std::pow(x - x_int, 2) + std::pow(y - y_int, 2));
            //     int_vec.push_back(dist);
            //     // ROS_ERROR_STREAM("DWALL: " << dist);
            // }
            
            
        }
        int_vec.push_back(range_max);

        
        scan_flag = true;
        double min = *min_element(int_vec.begin(), int_vec.end());
        laser.ranges[j] = min;
        ang += angle_inc;
        if (ang > angle_max)
        {
            ang = 0.0;
        }
        if (min < 3.4)
        {
            // ROS_ERROR_STREAM("ANGLE " << ang);
            // ROS_ERROR_STREAM("-----------------");
            // ROS_ERROR_STREAM("VECTOR: ");
            // ROS_ERROR_STREAM("-----------------");
            // ROS_ERROR_STREAM("MIN: " << min);
            for (int k=0; k<int_vec.size();k++)
            {
                ROS_ERROR_STREAM(int_vec[k]);
            }
            
        }
        // ROS_ERROR_STREAM("LASER MSG " << laser);
    }

}

int main(int argc, char * argv[])
{

    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh_prv("~");
    ros::NodeHandle nh;
    
    nh_prv.param("rate", rate, 500);
    ros::Rate r(rate);

    nh_prv.param("x0", x_0, 0.0);
    nh_prv.param("y0", y_0, 0.0);
    nh_prv.param("theta0", theta_0, 0.0);
    
    nh_prv.param("x_length", x_length, 5.0);
    nh_prv.param("y_length", y_length, 5.0);

    nh_prv.getParam("obstacles/obs_x", obs_x);
    nh_prv.getParam("obstacles/obs_y", obs_y);
    nh_prv.getParam("obstacles/radius", radius);
    nh_prv.getParam("obstacles/height", height);

    nh_prv.getParam("vnoise_mean", vnoise_mean);
    nh_prv.getParam("vnoise_stddev", vnoise_stddev);
    nh_prv.getParam("slip_min", slip_min);
    nh_prv.getParam("slip_max", slip_max);

    nh_prv.getParam("sensor/angle_min", angle_min);
    nh_prv.getParam("sensor/angle_max", angle_max);
    nh_prv.getParam("sensor/angle_increment", angle_inc);
    nh_prv.getParam("sensor/time_increment", time_inc);
    nh_prv.getParam("sensor/scan_time", scan_time);
    nh_prv.getParam("sensor/range_min", range_min);
    nh_prv.getParam("sensor/range_max", range_max);
    nh_prv.getParam("sensor/num_samples", num_samples);


    nh.getParam("collision_radius", collision_rad);

    nh_prv.getParam("basic_sensor_variance", basic_sensor_variance);


    // get encoder ticks conversion param
    // if it doesn't exist, throw an error
    if (!nh.hasParam("motor_cmd_to_radsec"))
    {
        ROS_ERROR_STREAM("No param named 'motor_cmd_to_radsec'.");
        return 1;
    }
    else
    {
        nh.getParam("motor_cmd_to_radsec", cmd_to_radsec);
    }

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

    x = x_0;
    y = y_0;
    theta = theta_0;

    std_msgs::UInt64 step;

    ros::Publisher pub_step = nh_prv.advertise<std_msgs::UInt64>("timestep", 1000);
    // ros::Publisher pub_joints = nh.advertise<sensor_msgs::JointState>("red/joint_states", 1000);
    ros::Publisher marker_pub = nh_prv.advertise<visualization_msgs::MarkerArray>("obstacles", 1, true);
    ros::Publisher wall_pub = nh_prv.advertise<visualization_msgs::MarkerArray>("walls", 1, true);
    ros::Publisher sensor_pub = nh.advertise<nuturtlebot_msgs::SensorData>("sensor_data", 1000);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("robot_path", 1000);
    ros::Publisher fake_sensor_pub = nh.advertise<visualization_msgs::MarkerArray>("fake_sensor", 1000);
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("laser_scan", 1000);

    ros::Subscriber wheel_sub = nh.subscribe("wheel_cmd", 1000, wheelCallback);

    // sensor_msgs::JointState js;

    ros::ServiceServer reset = nh_prv.advertiseService("reset", resetCallback);
    ros::ServiceServer teleport = nh_prv.advertiseService("teleport", teleportCallback);

    ros::Timer fake_marker_timer = nh.createTimer(ros::Duration(0.2), marktimerCallback);
    ros::Timer scanner_timer = nh.createTimer(ros::Duration(0.2), scantimerCallback);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transform;
    tf2::Quaternion q;

    nav_msgs::Path path;
    geometry_msgs::PoseStamped ps;

    visualization_msgs::MarkerArray wall_ma;

    wall_ma.markers.resize(4);
    for (int i=0; i<4; i++)
    {
        if (i == 3)
        {
            //set header and timestamp
            wall_ma.markers[i].header.frame_id = "world";
            wall_ma.markers[i].header.stamp = ros::Time::now();

            //set id diff for each marker
            wall_ma.markers[i].id = i;

            //set color and action
            wall_ma.markers[i].type = visualization_msgs::Marker::CUBE;
            wall_ma.markers[i].action = visualization_msgs::Marker::ADD;

            //set pose of marker
            wall_ma.markers[i].pose.position.x = (x_length/2.0) + (w_thick/2.0);
            wall_ma.markers[i].pose.position.y = 0;
            wall_ma.markers[i].pose.position.z = 0;
            wall_ma.markers[i].pose.orientation.x = 0;
            wall_ma.markers[i].pose.orientation.y = 0;
            wall_ma.markers[i].pose.orientation.z = 0;
            wall_ma.markers[i].pose.orientation.w = 1;

            //set size
            wall_ma.markers[i].scale.x = w_thick;
            wall_ma.markers[i].scale.y = y_length;
            wall_ma.markers[i].scale.z = 0.25;


            //set color
            wall_ma.markers[i].color.r = 1.0;
            wall_ma.markers[i].color.g = 0.0;
            wall_ma.markers[i].color.b = 0.0;
            wall_ma.markers[i].color.a = 1.0;
        }

        if (i == 2)
        {
            //set header and timestamp
            wall_ma.markers[i].header.frame_id = "world";
            wall_ma.markers[i].header.stamp = ros::Time::now();

            //set id diff for each marker
            wall_ma.markers[i].id = i;

            //set color and action
            wall_ma.markers[i].type = visualization_msgs::Marker::CUBE;
            wall_ma.markers[i].action = visualization_msgs::Marker::ADD;

            //set pose of marker
            wall_ma.markers[i].pose.position.x = 0;
            wall_ma.markers[i].pose.position.y = (y_length/2.0) + (w_thick/2.0);
            wall_ma.markers[i].pose.position.z = 0;
            wall_ma.markers[i].pose.orientation.x = 0;
            wall_ma.markers[i].pose.orientation.y = 0;
            wall_ma.markers[i].pose.orientation.z = 0;
            wall_ma.markers[i].pose.orientation.w = 1;

            //set size
            wall_ma.markers[i].scale.x = x_length + 2*w_thick;
            wall_ma.markers[i].scale.y = w_thick;
            wall_ma.markers[i].scale.z = 0.25;


            //set color
            wall_ma.markers[i].color.r = 1.0;
            wall_ma.markers[i].color.g = 0.0;
            wall_ma.markers[i].color.b = 0.0;
            wall_ma.markers[i].color.a = 1.0;
        }

        if (i == 1)
        {
            //set header and timestamp
            wall_ma.markers[i].header.frame_id = "world";
            wall_ma.markers[i].header.stamp = ros::Time::now();

            //set id diff for each marker
            wall_ma.markers[i].id = i;

            //set color and action
            wall_ma.markers[i].type = visualization_msgs::Marker::CUBE;
            wall_ma.markers[i].action = visualization_msgs::Marker::ADD;

            //set pose of marker
            wall_ma.markers[i].pose.position.x = -(x_length/2.0) - (w_thick/2.0);
            wall_ma.markers[i].pose.position.y = 0;
            wall_ma.markers[i].pose.position.z = 0;
            wall_ma.markers[i].pose.orientation.x = 0;
            wall_ma.markers[i].pose.orientation.y = 0;
            wall_ma.markers[i].pose.orientation.z = 0;
            wall_ma.markers[i].pose.orientation.w = 1;

            //set size
            wall_ma.markers[i].scale.x = w_thick;
            wall_ma.markers[i].scale.y = y_length;
            wall_ma.markers[i].scale.z = 0.25;


            //set color
            wall_ma.markers[i].color.r = 1.0;
            wall_ma.markers[i].color.g = 0.0;
            wall_ma.markers[i].color.b = 0.0;
            wall_ma.markers[i].color.a = 1.0;
        }

        if (i == 0)
        {
            //set header and timestamp
            wall_ma.markers[i].header.frame_id = "world";
            wall_ma.markers[i].header.stamp = ros::Time::now();

            //set id diff for each marker
            wall_ma.markers[i].id = i;

            //set color and action
            wall_ma.markers[i].type = visualization_msgs::Marker::CUBE;
            wall_ma.markers[i].action = visualization_msgs::Marker::ADD;

            //set pose of marker
            wall_ma.markers[i].pose.position.x = 0;
            wall_ma.markers[i].pose.position.y = - (y_length/2.0) - (w_thick/2.0);
            wall_ma.markers[i].pose.position.z = 0;
            wall_ma.markers[i].pose.orientation.x = 0;
            wall_ma.markers[i].pose.orientation.y = 0;
            wall_ma.markers[i].pose.orientation.z = 0;
            wall_ma.markers[i].pose.orientation.w = 1;

            //set size
            wall_ma.markers[i].scale.x = x_length + 2*w_thick;
            wall_ma.markers[i].scale.y = w_thick;
            wall_ma.markers[i].scale.z = 0.25;


            //set color
            wall_ma.markers[i].color.r = 1.0;
            wall_ma.markers[i].color.g = 0.0;
            wall_ma.markers[i].color.b = 0.0;
            wall_ma.markers[i].color.a = 1.0;
        }

    }

    wall_pub.publish(wall_ma);

    visualization_msgs::MarkerArray ma;
    ma.markers.resize(obs_x.size());
    for (int i=0;i<int(obs_x.size());i++)
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

        transform.header.stamp = ros::Time::now();

        transform.header.frame_id = "world";
        transform.child_frame_id = "red_base_footprint";

        // ROS_ERROR_STREAM("X: " << x);
        // ROS_ERROR_STREAM("Y: " << y);
        // ROS_ERROR_STREAM("-----");
        // ROS_ERROR_STREAM("THETA: " << theta);

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

        ps.header.stamp = ros::Time::now();
        ps.header.frame_id = "world";

        ps.pose.position.x = x;
        ps.pose.position.y = y;

        path.header.stamp = ros::Time::now();
        path.header.frame_id = "world";

        path.poses.push_back(ps);

        path_pub.publish(path);

        // js.name = {"red_wheel_left_joint", "red_wheel_right_joint"};
        // js.position = {0.0, 0.0};
        // pub_joints.publish(js);


        // ROS_ERROR_STREAM("LWHEEL POS: " << lwheel_pos*ticks_to_rad);
        // ROS_ERROR_STREAM("RWHEEL POS: " << rwheel_pos*ticks_to_rad);
        nuturtlebot_msgs::SensorData sensor_data;
        sensor_data.right_encoder = rwheel_pos/ticks_to_rad;
        sensor_data.left_encoder = lwheel_pos/ticks_to_rad;

        // ROS_ERROR_STREAM("SENSOR DATA: " << sensor_data);

        sensor_pub.publish(sensor_data);
        // ROS_ERROR_STREAM("NUSIM -- SENSOR_DATA PUBLISHED");

        if (fake_sensor_flag)
        {
            fake_sensor_pub.publish(fake_ma);
            fake_sensor_flag = false;
        }

        if (scan_flag)
        {
            scan_pub.publish(laser);
            scan_flag = false;
        }

        timestep++;
        ros::spinOnce();
        r.sleep();
    }
    return 0;
    
}