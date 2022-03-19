#include <armadillo>
#include <cmath>
#include <iostream>
#include "turtlelib/circle_fit.hpp"
#include "turtlelib/rigid2d.hpp"
#include <ros/ros.h>
#include <tuple>

/// \file 
/// \brief Circle fitting implementation

namespace turtlelib
{
    circleFit::circleFit(std::vector<double> x, std::vector<double> y)
    {
        cluster_x = x;
        cluster_y = y;
        cluster_x_shifted = arma::vec(int(cluster_x.size()));
        cluster_y_shifted = arma::vec(int(cluster_y.size()));
        
        
        z = arma::vec(int(x.size()));
    }

    std::pair<double, double> circleFit::compute_centroid()
    {
        return std::make_pair(mean(cluster_x), mean(cluster_y));
    }

    std::pair<arma::vec, arma::vec> circleFit::shift()
    {
        // ROS_ERROR_STREAM("SHIFT");
        arma::vec new_x(int(cluster_x.size()), arma::fill::ones);
        arma::vec new_y(int(cluster_y.size()), arma::fill::ones);

        double cent_x = std::get<0>(compute_centroid());
        double cent_y = std::get<1>(compute_centroid());

        cluster_x_shifted = cluster_x - (new_x*cent_x);
        cluster_y_shifted = cluster_y - (new_y*cent_y);

        return std::make_pair(cluster_x_shifted, cluster_y_shifted);
    }

    arma::vec circleFit::calc_z()
    {
        // ROS_ERROR_STREAM("CALC z");
        for (int i=0; i<int(z.size()); i++)
        {
            z.at(i) = pow(cluster_x_shifted.at(i), 2) + pow(cluster_y_shifted.at(i), 2);
        }
        return z;
    }

    arma::mat circleFit::z_mat()
    {
        // ROS_ERROR_STREAM("Z MAT");
        arma::vec ones(int(z.size()), arma::fill::ones);
        arma::mat Z;

        Z = join_rows(z, cluster_x_shifted, cluster_y_shifted, ones);

        return Z;
    }

    arma::mat circleFit::moment_matrix()
    {
        int n = int(z.size());
        arma::mat M;

        M = (1/n)*(z_mat().t())*(z_mat());
        return M;
    }

    arma::mat circleFit::constraint_matrix()
    {
        arma::mat H(4, 4, arma::fill::eye);
        H(0, 0) = 8.0*(arma::mean(z));
        H(0, 3) = 2;
        H(3, 0) = 2;
        H(3, 3) = 0;
        return H;
    }

    arma::mat circleFit::constraint_matrix_inv()
    {
        // ROS_ERROR_STREAM("CONSTRAINT MAT");
        arma::mat H_inv(4, 4, arma::fill::eye);
        H_inv(0, 0) = 0;
        H_inv(0, 3) = 0.5;
        H_inv(3, 0) = 0.5;
        H_inv(3, 3) = -2*(mean(z));
        return H_inv;
    }

    std::tuple<arma::mat, arma::vec, arma::mat> circleFit::svd(arma::mat X)
    {
        // ROS_ERROR_STREAM("SVD");
        arma::mat U;
        arma::vec s;
        arma::mat V;

        arma::svd(U, s, V, X);
        return std::make_tuple(U, s, V);
    }

    arma::mat circleFit::calc_A()
    {
        // ROS_ERROR_STREAM("CALC A");
        arma::mat U;
        arma::vec s;
        arma::mat V;
        arma::vec A;
        std::tie(U, s, V) = svd(z_mat());

        if (s.back() < pow(10, -12))
        {
            A = V.col(3);
        }
        else
        {
            // ROS_ERROR_STREAM("S");
            // s.print();
            arma::mat s_mat = arma::diagmat(s);
            // ROS_ERROR_STREAM("S MAT");
            // s_mat.print();
            // ROS_ERROR_STREAM("V");
            // V.print();
            arma::mat Y = V*s_mat*V.t();
            // ROS_ERROR_STREAM("Y");
            // Y.print();
            arma::mat Q = Y*constraint_matrix_inv()*Y;
            // ROS_ERROR_STREAM("Q");
            // Q.print();

            arma::vec eigval;
            arma::mat eigvec;

            arma::vec A_star;

            arma::eig_sym(eigval, eigvec, Q);

            for (int i=0; i<int(eigval.size()); i++)
            {
                if (eigval.at(i) > 0)
                {
                    A_star = eigvec.col(i);
                    break;
                }
            }

            A = arma::solve(Y, A_star);
        }
        // A.print();
        // std::cout << A.at(0);
        // std::cout << A.at(1);
        return A;
    }

    std::tuple<double, double, double> circleFit::fit_circle()
    {
        // ROS_ERROR_STREAM("FIT CIRCLE");
        shift();
        calc_z();
        arma::vec A = calc_A();

        double a = (-A.at(1))/(2*A.at(0));
        double b = (-A.at(2))/(2*A.at(0));
        // std::cout << a;
        // std::cout << b;
        double r = sqrt((pow(A.at(1), 2) + pow(A.at(2), 2) - (4*A.at(0)*A.at(3)))/(4*pow(A.at(0), 2)));

        double c_x = a + arma::mean(cluster_x);
        double c_y = b + arma::mean(cluster_y);

        return std::make_tuple(c_x, c_y, r);
    }

    arma::vec circleFit::get_clusterx()
    {
        return cluster_x;
    }

    arma::vec circleFit::get_clustery()
    {
        return cluster_y;
    }

    arma::vec circleFit::get_clusterx_shifted()
    {
        return cluster_x;
    }

    arma::vec circleFit::get_clustery_shifted()
    {
        return cluster_y;
    }

    arma::vec circleFit::get_z()
    {
        return z;
    }

    bool circleFit::classify_circle()
    {
        auto p1_x = cluster_x.front();
        auto p2_x = cluster_x.back();

        auto p1_y = cluster_y.front();
        auto p2_y = cluster_y.back();

        std::vector<double> angle_vec;

        for (int i=1; i<(int(cluster_x.size())-1); i++)
        {
            auto p_x = cluster_x.at(i);
            auto p_y = cluster_y.at(i);

            Vector2D v1, v2, v;

            v1.x = p1_x - p_x;
            v1.y = p1_y - p_y;

            v2.x = p2_x - p_x;
            v2.y = p2_y - p_y;

            double ang = v.angle(v1, v2);
            angle_vec.push_back(ang);
        }

        double mean = arma::mean(arma::vec(angle_vec));
        double stddev = arma::stddev(arma::vec(angle_vec));
        // ROS_ERROR_STREAM("MEAN: " << rad2deg(mean));
        // ROS_ERROR_STREAM("STDDEV: " << stddev);
        return (stddev < 0.15 && (rad2deg(mean) >= 90.0 && rad2deg(mean) <= 135.0)); 
    }


}