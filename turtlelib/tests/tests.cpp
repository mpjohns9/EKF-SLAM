#include "catch_ros/catch.hpp"
#include "turtlelib/rigid2d.hpp"
#include<sstream>

/// \file
/// \brief Test file for Transform2D class


//Collaborators: Devesh Bhura, Marco Morales

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

    turtlelib::Twist2D V0, Vt;
    turtlelib::Vector2D v;
    double theta = turtlelib::deg2rad(90);
    v.x = 1.0;
    v.y = 1.0;
    turtlelib::Transform2D Tab(v,theta);
    Vt.x = 0.0;
    Vt.y = 0.0;
    Vt.ang = 0.0;
    V0 = Tab(Vt);

    REQUIRE(turtlelib::almost_equal(V0.x,Vt.x,0.01));
    REQUIRE(turtlelib::almost_equal(V0.y,Vt.y,0.01));
    REQUIRE(turtlelib::almost_equal(V0.ang,Vt.ang,0.01));
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

TEST_CASE("Test Both Translational and Rotational Transform", "[transform]") {//Marco Morales
    double angle_init = 90.0;
    turtlelib::Vector2D vector;
    vector.x = 2.0;
    vector.y = 3.0;
    turtlelib::Transform2D transform = turtlelib::Transform2D(vector,angle_init);
    turtlelib::Vector2D vec = transform.translation();
    double angle = transform.rotation();
    double x_ph = vec.x;
    double y_ph = vec.y;

    REQUIRE( angle == 90.0 );
    REQUIRE( x_ph == 2.0 );
    REQUIRE( y_ph == 3.0 );
}

TEST_CASE("Test Input stream", "[transform]") { //Marco Morales
    std::stringstream ss;

    SECTION( "Just numbers" ) {
        ss << 90 << ' '<< 0 << ' ' << 1;
        int deg,x,y;
        ss >> deg >>x>>y;
        REQUIRE(deg == 90);
        REQUIRE(x == 0);
        REQUIRE(y == 1);
    }

    SECTION( "Strings added" ) {
        ss.peek();
        ss << "deg: " << 90 << " x: "<< 0 << " y: " << 1;
        int deg,x,y;
        ss >> deg >>x>>y;
        REQUIRE(deg == 90);
        REQUIRE(x == 0);
        REQUIRE(y == 1);
    }
}

TEST_CASE("Test Output stream", "[transform]") { //Marco Morales
    std::stringstream ss;
    std::string s("deg: 90 x: 0 y: 1");
    ss << "deg: " << 90 << " x: "<< 0 << " y: " << 1; 
    REQUIRE(s == ss.str());
} 


