#include <armadillo>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
// #include <tuple>

/// \file 
/// \brief Circle fitting implementation

namespace turtlelib
{
    circleFit::circleFit(std::vector<std::vector<double>> x, std::vector<std::vector<double>> y)
    {
        for (int i=0; i<int(x.size()); i++)
        {
            arma::vec x_arm = x.at(i);
            arma::vec y_arm = y.at(i);
            arma::join_cols(cluster_x, x_arm);
            arma::join_cols(cluster_y, y_arm);
        }
        arma::vec z(int(x.size()));
    }

    std::pair circleFit::compute_centroid(arma::vec x, arma::vec y)
    {
        return std::make_pair(mean(x), mean(y));
    }

    std::pair circleFit::shift(double cent_x, double cent_y)
    {
        arma::vec new_x(int(cluster_x.size()), arma::fill::ones);
        arma::vec new_y(int(cluster_x.size()), arma::fill::ones);

        cluster_x = cluster_x - (new_x*cent_x);
        cluster_y = cluster_x - (new_x*cent_x);

        return std::make_pair(cluster_x, cluster_y);
    }

    arma::vec circleFit::calc_z()
    {
        for (int i=0; i<int(z.size()); i++)
        {
            z.at(i) = pow(cluster_x.at(i), 2) + pow(cluster_y.at(i), 2);
        }
        return z;
    }

    // arma::mat circleFit::z_mat()
    // {
        
    // }
}