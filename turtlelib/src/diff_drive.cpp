#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include <stdexcept>
#include <iostream>
#include <tuple>
#include "ros/ros.h"

/// \file 
/// \brief Kinematic modeling differential drive robot implementation

namespace turtlelib
{
    diffDrive::diffDrive() 
    {
        config = {0, 0, 0};
        wheel_pos = {0.0, 0.0};
        wheel_vel = {0.0, 0.0};
    }

    diffDrive::diffDrive(Config c)
    {
        config = c;
    }

    diffDrive::diffDrive(Config c, WheelPos w_p, WheelVel w_v)
    {
        config = c;
        wheel_pos = w_p;
        wheel_vel = w_v;
    }

    std::tuple<Config, Twist2D> diffDrive::fwd_kin(WheelPos pos)
    {       
        // ROS_ERROR_STREAM("******************************");
        // ROS_ERROR_STREAM("FWD KIN");
        double l_vel = pos.l_pos - wheel_pos.l_pos;
        double r_vel = pos.r_pos - wheel_pos.r_pos;

        

        wheel_pos.l_pos = pos.l_pos;
        wheel_pos.r_pos = pos.r_pos;

        double dtheta = r*(r_vel - l_vel)/(2.0*D); // EQUATION 3 FROM Kinematics.pdf
        double vx = r*(r_vel + l_vel)/2.0; // EQUATION 4 FROM Kinematics.pdf

        // ROS_ERROR_STREAM("r-l: " << r_vel-l_vel);

        Twist2D V;
        V.ang = dtheta;
        V.x = vx;
        V.y = 0.0;
        // ROS_ERROR_STREAM("V DIFF DRIVE: " << V);
        // ROS_ERROR_STREAM("******************************");

        Vector2D old_v;
        old_v.x = config.x;
        old_v.y = config.y;

        Transform2D T_bbp = integrate_twist(V);

        Transform2D Twb(old_v, config.ang);

        Transform2D Twbp = Twb*T_bbp;

        config.ang = Twbp.rotation();
        config.x = Twbp.translation().x;
        config.y = Twbp.translation().y;

        

        // std::cout << "ANGLE " << T_bbp.rotation() << std::endl;
        // std::cout << "X " << T_bbp.translation().x << std::endl;
        // std::cout << "Y " << T_bbp.translation().y << std::endl;

        Config c;
        c.ang = Twbp.rotation();
        Vector2D trans = Twbp.translation();

        c.x = trans.x;
        c.y = trans.y;

        return std::make_tuple(c, V);
    }

    WheelVel diffDrive::inv_kin(Twist2D V)
    {
        if (almost_equal(V.y, 0.0))
        {
            WheelVel vel;

            vel.l_vel = (-D*V.ang + V.x)/r; // EQUATION 1 FROM Kinematics.pdf
            vel.r_vel = (D*V.ang + V.x)/r; // EQUATION 2 FROM Kinematics.pdf

            return vel;
        }
        else
        {
            throw std::logic_error("Invlaid twist -- cannot be accomplished without slipping.");
        }
    }


}