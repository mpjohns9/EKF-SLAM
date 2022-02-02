#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP

#include "rigid2d.hpp"

/// \file
/// \brief Models the kinematics of a differential drive robot

namespace turtlelib
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846; 
    constexpr double D = 0.08;
    constexpr double r = 0.033;

    struct Config {
        double ang = 0.0;
        double x = 0.0;
        double y = 0.0;
    };

    struct WheelPos {
        double l_pos = 0.0;
        double r_pos = 0.0;
    };

    struct WheelVel {
        double l_vel = 0.0;
        double r_vel = 0.0;
    };

    class diffDrive
    {
    public:
        Config fwd_kin(WheelPos pos);
        WheelPos inv_kin(Twist2D V);
    private:
        Config config;
        WheelPos wheel_pos;
    };

}

#endif