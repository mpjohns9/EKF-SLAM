#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP

#include "rigid2d.hpp"

/// \file
/// \brief Models the kinematics of a differential drive robot

namespace turtlelib
{
    // /// \brief Track width
    // constexpr double D = 0.08;

    // /// \brief Wheel radius
    // constexpr double r = 0.033;

    /// \brief configuration of robot
    struct Config {
        /// \brief current orientation
        double ang = 0.0;

        /// \brief the x-coordinate
        double x = 0.0;

        /// \brief the y-coordinate
        double y = 0.0;
    };

    /// \brief position of left and right wheels
    struct WheelPos {
        /// \brief position in radians of L wheel
        double l_pos = 0.0;

        /// \brief position in radians of R wheel
        double r_pos = 0.0;
    };

    /// \brief velocity of left and right wheels
    struct WheelVel {
        /// \brief velocity of left wheel
        double l_vel = 0.0;

        /// \brief velocity of right wheel
        double r_vel = 0.0;
    };

    /// \brief kinematics of differential drive robot
    class diffDrive
    {
    public:
        diffDrive();

        explicit diffDrive(Config c);

        explicit diffDrive(Config c, WheelPos w_p, WheelVel w_v);

        /// \brief calculates new config given wheel position
        /// \param pos - position of wheels
        /// \return new robot configuration
        Config fwd_kin(WheelPos pos);

        /// \brief calculates wheel velocities required to make robot
        /// move at a given twist
        /// \param V - given twist
        /// \return wheel velocities of robot
        WheelVel inv_kin(Twist2D V);

    private:
        Config config;
        WheelPos wheel_pos;
        WheelVel wheel_vel;
        double D = 0.08;
        double r = 0.033;
    };

}

#endif