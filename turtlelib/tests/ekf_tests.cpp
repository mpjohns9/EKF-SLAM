#include "catch_ros/catch.hpp"
#include "turtlelib/ekf.hpp"

using namespace turtlelib;

TEST_CASE("Constructor test", "[slam]")
{
    turtlelib::Config c {1.0, 1.0, 1.0};
    std::vector<double> obs_x = {2.0, 2.0};
    std::vector<double> obs_y = {3.0, 3.0};

    turtlelib::EKF e = turtlelib::EKF(c, obs_x, obs_y, 2);

    CHECK(e.map_state()[0] == Approx(1.0));
    CHECK(e.map_state()[1] == Approx(1.0));
    CHECK(e.map_state()[2] == Approx(1.0));
    CHECK(e.map_state()[3] == Approx(2.0));
    CHECK(e.map_state()[4] == Approx(3.0));
    CHECK(e.map_state()[5] == Approx(2.0));
    CHECK(e.map_state()[6] == Approx(2.0));
    CHECK(e.map_state().n_elem == 7);
}