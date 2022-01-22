#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "turtlelib/rigid2d.hpp"

TEST_CASE("translation", "[vector]") {
    turtlelib::Vector2D v;
    v.x = 1;
    v.y = 2;

    turtlelib::Transform2D tf = turtlelib::Transform2D(v);
    turtlelib::Vector2D v_2 = tf.translation();

    REQUIRE(v_2.x == Approx(1));
    REQUIRE(v_2.y == Approx(2));

}

TEST_CASE("rotation", "[angle]") {
    double ang;
    ang = 1;

    turtlelib::Transform2D tf = turtlelib::Transform2D(ang);
    double rads = tf.rotation();

    REQUIRE(rads == 1);
}

TEST_CASE("cons_ang", "[cons_ang]") {
    double ang;
    ang = 1;

    turtlelib::Transform2D tf = turtlelib::Transform2D(ang);
    double rads = tf.rotation();

    REQUIRE(rads == 1);
}

TEST_CASE("cons_v", "[cons_v]") {
    turtlelib::Vector2D v;
    v.x = 1;
    v.y = 2;

    turtlelib::Transform2D tf = turtlelib::Transform2D(v);
    turtlelib::Vector2D v_2 = tf.translation();

    REQUIRE(v_2.x == Approx(1));
    REQUIRE(v_2.y == Approx(2));

}



