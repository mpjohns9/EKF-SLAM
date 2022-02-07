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

        dtheta = r*(pos.r_pos - pos.l_pos)/2*D;
        vx = r*(pos.r_pos + pos.l_pos)/2;

        Twist2D V;
        V.ang = dtheta;
        V.x = vx;
        V.y = 0.0;

        Vector2D old_v;
        old_v.x = config.x;
        old_v.y = config.y;

        Transform2D T;
        Transform2D new_T = T.integrate_twist(V);

        Vector2D new_v = new_T(old_v);

        Config c;
        c.ang = new_T.rotation() + config.ang;
        c.x = new_v.x + old_v.x;
        c.y = new_v.y + old_v.y;

        return c;
    }

    WheelVel diffDrive::inv_kin(Twist2D V)
    {
        if (V.y == 0.0)
        {
            WheelVel vel;

            vel.l_vel = (-D*V.ang + V.x)/r;
            vel.r_vel = (D*V.ang + V.x)/r;

            return vel;
        }
        else
        {
            throw std::logic_error("Invlaid twist -- cannot be accomplished without slipping.");
        }
    }


}