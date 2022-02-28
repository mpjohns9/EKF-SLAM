#ifndef EKF_INCLUDE_GUARD_HPP
#define EKF_INCLUDE_GUARD_HPP

/// \file
/// \brief Extended Kalman Filter library

#include<armadillo>
#include"diff_drive.hpp"

namespace turtlelib
{
    class EKF
    {
    public:
        EKF();

        explicit EKF(Config s);

        explicit EKF(Config s, std::vector<double> obs_x, std::vector<double> obs_y, int n);

        // arma::mat get_h(int j);

        arma::vec map_state();


    
    private:
        int num_obstacles;
        Config state;
        arma::vec obstacles;
        arma::vec q;
        arma::vec m;
    };
}

#endif