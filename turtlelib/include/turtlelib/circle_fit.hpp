#ifndef CIRCLE_INCLUDE_GUARD_HPP
#define CIRCLE_INCLUDE_GUARD_HPP

/// \file
/// \brief Circle fitting library

#include<armadillo>

namespace 
{
    class circleFit
    {
    public:
        std::pair circleFit(std::vector<std::vector<double>> x, std::vector<std::vector<double>> y);
        std::pair compute_centroid(arma::vec x, arma::vec y);
        std::pair shift(double cent_x, double cent_y);


    private:
        arma::vec cluster_x;
        arma::vec cluster_y;
        arma::vec z;
    };
}

#endif