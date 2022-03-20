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

        /// \brief initialize EKF variables for n landmarks
        /// \param n - max number of landmarks expected
        explicit EKF(int n);

        /// \brief initialize EKF variables for n landmarks and robot config
        /// \param n - max number of landmarks expected
        /// \param s - robot configuration
        explicit EKF(Config s, int n);

        /// \brief initialize landmarks
        /// \param obs_x - x coordinates of landmarks
        /// \param obs_y - y coordinates of landmarks
        /// \return vector containing currently detected obstacles
        arma::vec initialize_landmarks(std::vector<double> obs_x, std::vector<double> obs_y);

        /// \brief calculate range bearing measurements
        /// \param j - landmark
        /// \return vector containing range bearing measurements
        arma::vec calc_h(int j);

        /// \brief calculate jacobian of range bearing vector
        /// \param j - landmark
        /// \return jacobian matrix of h
        arma::mat calc_H(int j);

        /// \brief calculate A matrix
        /// \param u - velocity twist
        /// \return A matrix
        arma::mat calc_A(Twist2D u);

        /// \brief update state estimate
        /// \param u - velocity twist
        /// \return vector containing current state estimate
        arma::vec update_state(Twist2D u);

        /// \brief get current obstacles
        /// \return vector containing map obstacles
        arma::vec map_state();

        /// \brief get covariance 
        /// \return tuple of covariance matrices
        std::tuple<arma::mat, arma::mat, arma::mat> sigmas();

        /// \brief get robot configuration estimate
        /// \return robot config from state estimate
        Config config();

        /// \brief extract obstacles from state vector
        /// \return obstacle vector
        std::vector<double>  obs_vec();

        /// \brief update estimate using model
        /// \param u - twist
        void predict(Twist2D u);
        
        /// \brief update step
        /// \param j - landmark
        /// \param x - x coordinate of landmark
        /// \param y - y coordinate of landmark
        void update(int j, double x, double y);

        /// \brief check known obstacles for data association
        /// \param x - x coordinate of landmark
        /// \param y - y coordinate of landmark
        /// \return true if landmark exists, false if new landmark
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