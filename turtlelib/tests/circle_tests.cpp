#include "catch_ros/catch.hpp"
#include "turtlelib/circle_fit.hpp"
#include <tuple>

using namespace turtlelib;

TEST_CASE("Circle tests", "[landmarks]")
{
    SECTION("Test 1")
    {
        std::vector<double> x;

        x = {1.0, 2.0, 5.0, 7.0, 9.0, 3.0};

        std::vector<double> y;

        y = {7.0, 6.0, 8.0, 7.0, 5.0, 7.0};

        turtlelib::circleFit cf = turtlelib::circleFit(x, y);

        double c_x, c_y, r;

        std::tie(c_x, c_y, r) = cf.fit_circle();

        CHECK(c_x == Approx(4.615482));
        CHECK(c_y == Approx(2.807354));
        CHECK(r == Approx(4.8275));

        // CHECK(size(cf.calc_A())[0] == 3);
    }

    // SECTION("Test 2")
    // {
    //     std::vector<double> x;

    //     x = {-1.0, -0.3, 0.3, 1.0};

    //     std::vector<double> y;

    //     y = {0.0, -0.06, 0.1, 0.0};

    //     turtlelib::circleFit cf = turtlelib::circleFit(x, y);

    //     double c_x, c_y, r;

    //     std::tie(c_x, c_y, r) = cf.fit_circle();

    //     CHECK(c_x == Approx(0.4908357));
    //     CHECK(c_y == Approx(-22.15212));
    //     CHECK(r == Approx(22.17979));
    // }

    // SECTION("Compute centroid")
    // {
    //     std::vector<double> x;

    //     x = {1.0, 2.0, 3.0};

    //     std::vector<double> y;

    //     y = {1.0, 2.0, 3.0};

    //     turtlelib::circleFit cf = turtlelib::circleFit(x, y);


    //     double c_x, c_y;
    //     std::tie(c_x, c_y) = cf.compute_centroid();

    
    //     CHECK(c_x == Approx(2.0));
    //     CHECK(c_y == Approx(2.0));
    // }

    // SECTION("Shift test")
    // {
    //     std::vector<double> x;

    //     x = {1.0, 2.0, 3.0};

    //     std::vector<double> y;

    //     y = {1.0, 2.0, 3.0};

    //     turtlelib::circleFit cf = turtlelib::circleFit(x, y);


    //     arma::vec x_shift, y_shift;
    //     std::tie(x_shift, y_shift) = cf.shift();

    
    //     CHECK(x_shift.at(0) == -1.0);
    //     CHECK(y_shift.at(0) == -1.0);
    //     CHECK(x_shift.at(1) == 0.0);
    //     CHECK(y_shift.at(1) == 0.0);
    //     CHECK(x_shift.at(2) == 1.0);
    //     CHECK(y_shift.at(2) == 1.0);
    // }
    
    // SECTION("z test")
    // {
    //     std::vector<double> x;

    //     x = {1.0, 2.0, 3.0};

    //     std::vector<double> y;

    //     y = {1.0, 2.0, 3.0};

    //     turtlelib::circleFit cf = turtlelib::circleFit(x, y);

    //     CHECK(cf.get_clusterx().at(0) == 1.0);
    //     CHECK(cf.get_clusterx().at(1) == 2.0);

    //     cf.shift();

    //     CHECK(cf.get_clusterx_shifted().at(0) == -1.0);
    //     CHECK(cf.get_clusterx_shifted().at(1) == 0.0);

    //     CHECK(cf.calc_z().at(0) == 2.0);
    //     CHECK(cf.calc_z().at(1) == 0.0);
    //     CHECK(cf.calc_z().at(2) == 2.0);
        
    // }

    // SECTION("Z matrix test")
    // {
    //     std::vector<double> x;

    //     x = {1.0, 2.0, 3.0};

    //     std::vector<double> y;

    //     y = {1.0, 2.0, 3.0};

    //     turtlelib::circleFit cf = turtlelib::circleFit(x, y);

    //     cf.shift();
    //     cf.calc_z();

    //     CHECK(cf.get_z().at(0) == 2.0);
    //     CHECK(cf.get_z().at(1) == 0.0);
    //     CHECK(cf.get_z().at(2) == 2.0);

    //     CHECK(size(cf.z_mat())[0] == 3);
    //     CHECK(size(cf.z_mat())[1] == 4);

    //     CHECK(cf.z_mat()(0, 0) == 2.0);
    //     CHECK(cf.z_mat()(0, 1) == -1.0);
    //     CHECK(cf.z_mat()(0, 2) == -1.0);
    //     CHECK(cf.z_mat()(0, 3) == 1.0);
    //     CHECK(cf.z_mat()(1, 0) == 0.0);
    //     CHECK(cf.z_mat()(1, 1) == 0.0);
    //     CHECK(cf.z_mat()(1, 2) == 0.0);
    //     CHECK(cf.z_mat()(1, 3) == 1.0);
    //     CHECK(cf.z_mat()(2, 0) == 2.0);
    //     CHECK(cf.z_mat()(2, 1) == 1.0);
    //     CHECK(cf.z_mat()(2, 2) == 1.0);
    //     CHECK(cf.z_mat()(2, 3) == 1.0);
        
    // }

    // SECTION("H inverse test")
    // {
    //     std::vector<double> x;

    //     x = {1.0, 2.0, 3.0};

    //     std::vector<double> y;

    //     y = {1.0, 2.0, 3.0};

    //     turtlelib::circleFit cf = turtlelib::circleFit(x, y);

    //     cf.shift();
    //     cf.calc_z();

    //     CHECK(cf.constraint_matrix_inv()(0, 0) == 0.0);
    //     CHECK(cf.constraint_matrix_inv()(0, 3) == 0.5);
    //     CHECK(cf.constraint_matrix_inv()(3, 3) == Approx(-2.66666667));
    //     CHECK(cf.constraint_matrix_inv()(3, 0) == 0.5);
    // }
}

