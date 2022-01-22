#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "turtlelib/rigid2d.hpp"

TEST_CASE("translation", "[vector]") { //Marshall Johnson
    turtlelib::Vector2D v;
    v.x = 1;
    v.y = 2;

    turtlelib::Transform2D tf = turtlelib::Transform2D(v);
    turtlelib::Vector2D v_2 = tf.translation();

    REQUIRE(v_2.x == Approx(1));
    REQUIRE(v_2.y == Approx(2));

}

TEST_CASE("rotation", "[angle]") {//Marshall Johnson
    double ang;
    ang = 1;

    turtlelib::Transform2D tf = turtlelib::Transform2D(ang);
    double rads = tf.rotation();

    REQUIRE(rads == 1);
}

TEST_CASE("cons_ang", "[cons_ang]") {//Marshall Johnson
    double ang;
    ang = 1;

    turtlelib::Transform2D tf = turtlelib::Transform2D(ang);
    double rads = tf.rotation();

    REQUIRE(rads == 1);
}

TEST_CASE("cons_v", "[cons_v]") {//Marshall Johnson
    turtlelib::Vector2D v;
    v.x = 1;
    v.y = 2;

    turtlelib::Transform2D tf = turtlelib::Transform2D(v);
    turtlelib::Vector2D v_2 = tf.translation();

    REQUIRE(v_2.x == Approx(1));
    REQUIRE(v_2.y == Approx(2));

}

TEST_CASE( "Test for Identity", "Identity" ) {// Devesh Bhura

    turtlelib::Vector2D v0;
    turtlelib::Transform2D Tab;

    v0 = Tab.translation();
    double theta_check = Tab.rotation();
    REQUIRE(turtlelib::almost_equal(theta_check,0.0,0.01));
    REQUIRE(turtlelib::almost_equal(v0.x,0.0,0.01));
    REQUIRE(turtlelib::almost_equal(v0.y,0.0,0.01));
}
TEST_CASE( "Test for inv", "[inverse]" ) {// Devesh Bhura

    double theta = turtlelib::deg2rad(90);
    turtlelib::Vector2D v, v0;
    v.x = 0;
    v.y = 0;
    turtlelib::Transform2D Tab(v,theta), Tba;

    v0 = Tab(v);


    REQUIRE(turtlelib::almost_equal(v.x,v0.x,0.1));
    REQUIRE(turtlelib::almost_equal(v.y,v0.y,0.1));
}


TEST_CASE( "Test for vector transformation", "Vector Transform" ) {// Devesh Bhura

    turtlelib::Vector2D v, v0, vt;
    double theta = turtlelib::deg2rad(90);
    v.x = 1.0;
    v.y = 1.0;
    turtlelib::Transform2D Tab(v,theta);
    vt.x = 0.0;
    vt.y = 0.0;
    v0 = Tab(vt);

    REQUIRE(turtlelib::almost_equal(v0.x,v.x,0.01));
    REQUIRE(turtlelib::almost_equal(v0.y,v.y,0.01));
}

TEST_CASE( "Test for Twist transformation", "Twist Transform" ) {// Devesh Bhura

    turtlelib::Twist V0, Vt;
    turtlelib::Vector2D v;
    double theta = turtlelib::deg2rad(90);
    v.x = 1.0;
    v.y = 1.0;
    turtlelib::Transform2D Tab(v,theta);
    Vt.xdot = 0.0;
    Vt.ydot = 0.0;
    Vt.thetadot = 0.0;
    V0 = Tab(Vt);

    REQUIRE(turtlelib::almost_equal(V0.xdot,Vt.xdot,0.01));
    REQUIRE(turtlelib::almost_equal(V0.ydot,Vt.ydot,0.01));
    REQUIRE(turtlelib::almost_equal(V0.thetadot,Vt.thetadot,0.01));
}


TEST_CASE( "Test for Multiplication", "Multiplication" ) {// Devesh Bhura

    turtlelib::Vector2D v, v0, vt;
    double theta0;
    double theta = turtlelib::deg2rad(90);
    v.x = 1.0;
    v.y = 1.0;
    vt.x = 0.0;
    vt.y = 0.0;
    turtlelib::Transform2D Tab(v,theta), Tbc(vt,0);

    Tab*=Tbc;

    v0 = Tab.translation();
    theta0 = Tab.rotation();


    REQUIRE(turtlelib::almost_equal(theta0, theta,0.01));
    REQUIRE(turtlelib::almost_equal(v0.x, v.x,0.01));
    REQUIRE(turtlelib::almost_equal(v0.y, v.y,0.01));
}



