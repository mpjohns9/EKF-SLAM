#include "catch_ros/catch.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace turtlelib;

/// \file
/// \brief Test file for diffDrive class

// Collaborators: Marco Morales & Devesh Bhura

TEST_CASE("Integrate twist -- translation","[diffDrive]"){ 

    turtlelib::Twist2D V;
    V.x = 1.0;
    V.y = 1.0;

    turtlelib::Transform2D T;
    T = turtlelib::integrate_twist(V);

    turtlelib::Vector2D v;
    v = T.translation();
    double ang = T.rotation();

    REQUIRE(turtlelib::almost_equal(v.x, V.x, 0.01));
    REQUIRE(turtlelib::almost_equal(v.y, V.y, 0.01));
    REQUIRE(turtlelib::almost_equal(ang, 0.0, 0.01));

}

TEST_CASE("Integrate twist -- rotation","[diffDrive]"){ 

    turtlelib::Twist2D V;
    V.ang = 1.0;

    turtlelib::Transform2D T;
    T = turtlelib::integrate_twist(V);

    turtlelib::Vector2D v;
    v = T.translation();
    double ang = T.rotation();

    REQUIRE(turtlelib::almost_equal(v.x, 0.0, 0.01));
    REQUIRE(turtlelib::almost_equal(v.y, 0.0, 0.01));
    REQUIRE(turtlelib::almost_equal(ang, V.ang, 0.01));

}

TEST_CASE("Integrate twist -- both","[diffDrive]"){ 

    turtlelib::Twist2D V;
    V.x = 1.0;
    V.y = 1.0;

    double PI = turtlelib::PI;
    V.ang = PI;

    turtlelib::Transform2D T;
    T = turtlelib::integrate_twist(V);

    turtlelib::Vector2D v;
    v = T.translation();
    double ang = T.rotation();

    CHECK(v.x == Approx(-0.6366197724));
    CHECK(v.y == Approx(0.6366197724));
    CHECK(ang == Approx(PI));

}

TEST_CASE("FKin -- forward", "[diffDrive]") {
    Config c, new_c;
    WheelPos phi, phi_new, phi_old;
    WheelVel phi_dot;

    c.x = 0;
    c.y = 0;
    c.ang = 0;

    phi.l_pos = 0;
    phi.r_pos = 0;

    phi_dot.l_vel = 0;
    phi_dot.r_vel = 0;

    diffDrive dd = diffDrive(c, phi, phi_dot);

    phi_new.l_pos = PI/4;
    phi_new.r_pos = PI/4;

    new_c = std::get<0>(dd.fwd_kin(phi_new));

    CHECK(new_c.x == Approx(0.033*(PI/4))); 
    CHECK(new_c.y == Approx(0));
    CHECK(new_c.ang == Approx(0)); 
}

TEST_CASE("FKin -- rotation", "[diffDrive]") {
    Config c,new_c;
    WheelPos phi, phi_new, phi_old;
    WheelVel phi_dot;

    c.x = 0;
    c.y = 0;
    c.ang = 0;

    phi.l_pos = 0;
    phi.r_pos = 0;

    phi_dot.l_vel = 0;
    phi_dot.r_vel = 0;

    diffDrive dd = diffDrive(c, phi, phi_dot);

    phi_new.l_pos = PI/4;
    phi_new.r_pos = -PI/4;

    new_c = std::get<0>(dd.fwd_kin(phi_new));

    CHECK(new_c.x == Approx(0.0)); 
    CHECK(new_c.y == Approx(0));
    CHECK(new_c.ang == Approx(-0.3239767424)); 
}


TEST_CASE("FKin -- arc", "[DiffDrive]") {
    Config c, new_c;
    WheelPos phi, phi_new, phi_old;
    WheelVel phi_dot;

    c.x = 0;
    c.y = 0;
    c.ang = 0;

    phi.l_pos = 0;
    phi.r_pos = 0;

    phi_dot.l_vel = 0;
    phi_dot.r_vel = 0;

    diffDrive dd = diffDrive(c, phi, phi_dot);

    phi_new.l_pos = 19.992;
    phi_new.r_pos = 27.6079;

    new_c = std::get<0>(dd.fwd_kin(phi_new));

    CHECK(new_c.x == Approx(0.5)); 
    CHECK(new_c.y == Approx(0.5));
    CHECK(new_c.ang == Approx(PI/2)); 
}

TEST_CASE("IKin -- forward", "[DiffDrive]") {
    diffDrive dd;
    Twist2D t;
    WheelVel v;

    t.x = 1;
    t.y = 0;
    t.ang = 0;

    v = dd.inv_kin(t);

    CHECK(v.l_vel == Approx(30.303030303)); 
    CHECK(v.r_vel == Approx(30.303030303)); 
}

TEST_CASE("IKin -- backward", "[diffDrive]") {
    diffDrive dd;
    Twist2D t;
    WheelVel v;

    t.x = -1;
    t.y = 0;
    t.ang = 0;

    v = dd.inv_kin(t);  

    CHECK(v.l_vel == Approx(-30.303030303)); 
    CHECK(v.r_vel == Approx(-30.303030303)); 
}


TEST_CASE("IKin -- rotation", "[DiffDrive]") {
    diffDrive dd;
    Twist2D t;
    WheelVel v;

    t.x = 0;
    t.y = 0;
    t.ang = PI/2;

    v = dd.inv_kin(t);

    CHECK(v.l_vel == Approx(-3.8079910953)); 
    CHECK(v.r_vel == Approx(3.8079910953));
}

TEST_CASE("Impossible twist", "[diffDrive]") { 
    Twist2D V;

    V.x = 1.0;
    V.y = 1.0;
    V.ang = 1.0;

    diffDrive dd;

    CHECK_THROWS(dd.inv_kin(V));

}