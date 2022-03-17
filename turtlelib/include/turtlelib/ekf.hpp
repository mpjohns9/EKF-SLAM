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

        explicit EKF(int n);

        explicit EKF(Config s, int n);

        arma::vec initialize_landmarks(std::vector<double> obs_x, std::vector<double> obs_y);

        arma::vec calc_h(int j);

        arma::mat calc_H(int j);

        arma::mat calc_A(Twist2D u);

        arma::vec update_state(Twist2D u);

        arma::vec map_state();

        std::tuple<arma::mat, arma::mat, arma::mat> sigmas();

        Config config();

        std::vector<double>  obs_vec();

        void predict(Twist2D u);
        
        void update(int j, double x, double y);

        bool check_known_obs(double x, double y);


    
    private:
        int num_obstacles;
        Config state;
        Config state_prev;
        arma::vec obstacles;
        arma::vec q;
        arma::vec q_prev;
        arma::vec xi_minus;
        arma::vec xi_plus;
        arma::vec xi_prev;
        arma::mat sigma_minus;
        arma::mat sigma_plus;
        arma::mat sigma_prev;
        std::vector<Vector2D> known_obs;
        // diffDrive::Twist2D u;
    };
}

#endif