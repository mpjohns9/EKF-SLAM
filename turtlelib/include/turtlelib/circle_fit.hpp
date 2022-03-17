#ifndef CIRCLE_INCLUDE_GUARD_HPP
#define CIRCLE_INCLUDE_GUARD_HPP

/// \file
/// \brief Circle fitting library

#include<armadillo>
#include<tuple>

namespace turtlelib
{
    class circleFit
    {
    public:
        explicit circleFit(std::vector<double> x, std::vector<double> y);

        std::pair<double, double> compute_centroid();
        std::pair<arma::vec, arma::vec> shift();
        arma::vec calc_z();
        arma::mat z_mat();
        arma::mat moment_matrix();
        arma::mat constraint_matrix();
        arma::mat constraint_matrix_inv();
        std::tuple<arma::mat, arma::vec, arma::mat> svd(arma::mat X);
        arma::mat calc_A();
        std::tuple<double, double, double> fit_circle();
        arma::vec get_clusterx();
        arma::vec get_clustery();
        arma::vec get_clusterx_shifted();
        arma::vec get_clustery_shifted();
        arma::vec get_z();

    private:
        arma::vec cluster_x;
        arma::vec cluster_y;
        arma::vec cluster_x_shifted;
        arma::vec cluster_y_shifted;
        arma::vec z;
    };
}

#endif