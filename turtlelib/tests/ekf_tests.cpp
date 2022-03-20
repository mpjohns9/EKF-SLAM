#include "catch_ros/catch.hpp"
#include "turtlelib/ekf.hpp"
// #include <tuple>

using namespace turtlelib;

TEST_CASE("Constructor test", "[slam]")
{
    turtlelib::Config c {0.0, 0.0, 0.0};
    int j = 3;
    // std::vector<double> obs_x = {2.0, 2.0};
    // std::vector<double> obs_y = {3.0, 3.0};

    turtlelib::EKF e = turtlelib::EKF(c, j);

    // turtlelib::Twist2D u {2.0, 2.0, 0.0};
    // double x = 5.0;
    // double y = 5.0;

    // std::tuple t = e.sigmas();

    // e.predict(u);
    // e.update(j, x, y);

    CHECK(int(e.map_state().n_elem) == 6);
    // CHECK(std::get<0>(t).n_rows == 9);
    // CHECK(std::get<0>(t).n_cols == 9);
    // CHECK(e.map_state().at(0) == Approx(1.0));
    // CHECK(e.map_state().at(1) == Approx(1.0));
    // CHECK(e.map_state().at(2) == Approx(1.0));
    // CHECK(e.map_state().at(3) == Approx(2.0));
    // CHECK(e.map_state().at(4) == Approx(3.0));
    // CHECK(e.map_state().at(5) == Approx(2.0));
    // CHECK(e.map_state().at(6) == Approx(3.0));
    // CHECK(e.map_state().n_elem == 7);
}

TEST_CASE("Calculate A", "[slam]")
{
    turtlelib::Config c {0.0, 0.0, 0.0};
    int j = 3;

    turtlelib::EKF e = turtlelib::EKF(c, j);

    turtlelib::Twist2D u {2.0, 2.0, 0.0};

    CHECK(e.calc_A(u).n_cols == 9);
    CHECK(e.calc_A(u).n_rows == 9);

}

// TEST_CASE("Calculate H", "[slam]")
// {
//     turtlelib::Config c {0.0, 0.0, 0.0};
//     int n = 3;
//     int j = 0;
    
//     turtlelib::EKF e = turtlelib::EKF(c, n);

//     turtlelib::Twist2D u {1.0, 1.0, 1.0};

//     arma::mat H = e.calc_H(j);

//     // CHECK(H.n_cols == 1);
//     CHECK(1 == 2);

// }