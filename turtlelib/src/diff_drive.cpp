#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include <stdexcept>

/// \file 
/// \brief Kinematic modeling differential drive robot implementation

namespace turtlelib
{
    Config diffDrive::fwd_kin(WheelPos pos)
    {        
        double dtheta = 0.0;
        double vx = 0.0;

        dtheta = r*(pos.r_wheel - pos.l_wheel)/2*D;
        vx = r*(ros.r_wheel + pos.l_wheel)/2;

        Twist2D V;
        V.ang = dtheta;
        V.x = vx;
        V.y = 0.0;

        Vector2D old_v;
        old_v.x = config.x;
        old_v.y = config.y;

        Transform2D new_T = Transform2D::integrate_twist(V);

        Vector2D new_v = new_T(old_v);

        c.ang = new_T.ang + ang;
        c.x = new_v.x + old_v.x;
        c.y = new_v.y + old_v.y;

        return c;
    }

    WheelPos inv_kin(Twist2D V)
    {
        if (V.y == 0.0)
        {
            WheelPos p;

            p.l_wheel = (-D*V.ang + V.x)/r;
            p.r_wheel = (D*V.ang + V.x)/r;

            return p;
        }
        else
        {
            throw std::logic_error("Invlaid twist -- cannot be accomplished without slipping.");
        }
    }


}