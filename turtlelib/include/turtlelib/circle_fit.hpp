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
        /// \brief Initialize cluster vectors
        /// \param x - x coordinates of cluster
        /// \param y - y coordinates of cluster
        explicit circleFit(std::vector<double> x, std::vector<double> y);

        /// \brief compute centroid of cluster
        /// \return pair containing x, y coords of centroid
        std::pair<double, double> compute_centroid();

        /// \brief shift coords so centroid is at origin
        /// \return x, y coords of shifted cluster
        std::pair<arma::vec, arma::vec> shift();

        /// \brief calculate z (x^2 + y^2)
        /// \return z vector
        arma::vec calc_z();

        /// \brief calculate z matrix
        /// \return matrix containing z, x, y vectors
        arma::mat z_mat();

        /// \brief calculate moment matrix
        /// \return moment matrix
        arma::mat moment_matrix();

        /// \brief calculate constraint matrix H
        /// \return constraint matrix
        arma::mat constraint_matrix();

        /// \brief calculate inverse of constraint matrix
        /// \return inverse of constraint matrix
        arma::mat constraint_matrix_inv();

        /// \brief compute Singular Value Decomposition of Z matrix
        /// \param X - Z matrix 
        /// \return tuple with U, s, V
        std::tuple<arma::mat, arma::vec, arma::mat> svd(arma::mat X);

        /// \brief calculate A matrix
        /// \return A matrix
        arma::mat calc_A();

        /// \brief calculate centroid and radius of circle
        /// \return x, y coordinates and radius of fitted circle
        std::tuple<double, double, double> fit_circle();

        /// \brief x coordinates of cluster
        /// \return x vector
        arma::vec get_clusterx();

        /// \brief y coordinates of cluster
        /// \return y vector
        arma::vec get_clustery();

        /// \brief shifted x coordinates of cluster
        /// \return x vector
        arma::vec get_clusterx_shifted();

        /// \brief shifted y coordinates of cluster
        /// \return shifted y vector
        arma::vec get_clustery_shifted();

        /// \brief z of cluster
        /// \return z vector
        arma::vec get_z();

        /// \brief classify as circle or not
        /// \return true if circle, otherwise false
        bool classify_circle();

    private:
        arma::vec cluster_x;
        arma::vec cluster_y;
        arma::vec cluster_x_shifted;
        arma::vec cluster_y_shifted;
        arma::vec z;
    };
}

#endif