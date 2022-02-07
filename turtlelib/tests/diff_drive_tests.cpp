#include "catch_ros/catch.hpp"
#include "turtlelib/diff_drive.hpp"

/// \file
/// \brief Test file for diffDrive class

TEST_CASE("impossible twist", "[twist]") { 
    turtlelib::Twist2D V;
    V.x = 1.0;
    V.y = 1.0;
    V.ang = 1.0;

    turtlelib::diffDrive dd;

    CHECK_THROWS(dd.inv_kin(V));

}