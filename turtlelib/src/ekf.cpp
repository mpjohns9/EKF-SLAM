#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"
#include <cmath>

/// \file 
/// \brief Extended Kalman Filter implementation

namespace turtlelib
{

    EKF::EKF(int n)
    {
        state = {0.0, 0.0, 0.0};
        q = {state.ang, state.x, state.y};
        num_obstacles = n;
        obstacles = arma::vec(2*n);
        xi_prev = arma::vec(3 + (2*n));
    }

    EKF::EKF(Config s, int n)
    {
        state = s;
        q = {state.ang, state.x, state.y};
        num_obstacles = n;
        obstacles = arma::vec(2*n);
        xi_prev = arma::vec(3 + (2*n), arma::fill::zeros);

        sigma_minus = arma::mat(3+(2*n), 3+(2*n), arma::fill::zeros);
        // arma::mat sig_m = arma::eye(2*n, 2*n);
        // sig_m = sig_m*INFINITY;
        // sigma_minus.submat(3, 3, 3+(2*n)-1, 3+(2*n)-1) = sig_m;

        sigma_prev = sigma_minus;
    }

    arma::vec EKF::initialize_landmarks(std::vector<double> obs_x, std::vector<double> obs_y)
    {
        int index = 0;
        for (int i=0; i<int(obs_x.size()); i++)
        {
            double r = sqrt(pow(obs_x.at(i), 2) + pow(obs_y.at(i), 2));
            double phi = atan2(obs_y.at(i), obs_x.at(i));

            double mx = state.x + r*cos(phi + state.ang);
            double my = state.y + r*sin(phi + state.ang);

            obstacles.at(index) = mx;
            obstacles.at(index+1) = my;
            index += 2;
        }

        xi_prev = arma::join_cols(q, obstacles);
    }

    arma::vec EKF::calc_h(int j)
    {
        double mx = xi_minus.at(3+(2*j));
        double my = xi_minus.at(4+(2*j));
        double r = sqrt(pow(mx - xi_minus.at(1), 2) + pow(my - xi_minus.at(2), 2));
        double phi = atan2(my - xi_minus.at(2), mx - xi_minus.at(1)) - xi_minus.at(0);

        arma::vec h = {r, phi};
        return h;
    }

    arma::mat EKF::calc_H(int j)
    {
        double mx = xi_minus.at(3+(2*j));
        double my = xi_minus.at(4+(2*j));

        double dx = mx - xi_minus.at(1);
        double dy = my - xi_minus.at(2);

        double d = pow(dx, 2) + pow(dy, 2);

        arma::mat H (2, 3 + (2*num_obstacles), arma::fill::zeros);

        arma::mat h_q = {
            {0, -dx/sqrt(d), -dy/sqrt(d)},
            {-1, dy/d, -dx/d}
        };

        arma::mat h_m = {
            {dx/sqrt(d), dy/sqrt(d)},
            {-dy/d, dx/d}
        };

        H.submat(0, 0, 1, 2) = h_q;
        H.submat(0, 3 + (2*(j-1)), 1, 4 + (2*(j-1))) = h_m;

        return H;
    }

    arma::mat EKF::calc_A(Twist2D u)
    {
        arma::mat I = arma::eye((3+(2*num_obstacles)), (3+(2*num_obstacles)));
        arma::mat mat((3+(2*num_obstacles)), (3+(2*num_obstacles)), arma::fill::zeros);

        if (almost_equal(u.ang, 0.0))
        {
            mat(1, 0) = -u.x*sin(xi_prev.at(0));
            mat(2, 0) = u.x*cos(xi_prev.at(0));
        }
        else
        {
            mat(1, 0) = ((-u.x/u.ang)*cos(xi_prev.at(0))) + ((u.x/u.ang)*cos(xi_prev.at(0) + u.ang));
            mat(2, 0) = ((-u.x/u.ang)*sin(xi_prev.at(0))) + ((u.x/u.ang)*sin(xi_prev.at(0) + u.ang));
        }

        arma::mat A = I + mat;
        return A;
    }

    arma::vec EKF::update_state(Twist2D u)
    {
        if (almost_equal(u.ang, 0.0))
        {
            arma::vec ut(3+(2*num_obstacles), arma::fill::zeros);
            ut.at(1) = u.x*cos(xi_prev.at(0));
            ut.at(2) = u.x*sin(xi_prev.at(0));

            xi_minus = xi_prev + ut;
            return xi_minus;
        }
        else
        {
            arma::vec ut(3+(2*num_obstacles), arma::fill::zeros);
            ut.at(0) = u.ang;
            ut.at(1) = ((-u.x/u.ang)*sin(xi_prev.at(0))) + ((u.x/u.ang)*sin(xi_prev.at(0) + u.ang));
            ut.at(2) = ((-u.x/u.ang)*cos(xi_prev.at(0))) + ((u.x/u.ang)*cos(xi_prev.at(0) + u.ang));

            xi_minus = xi_prev + ut;
            return xi_minus;
        }
    }

    arma::vec EKF::map_state()
    {
        return obstacles;
    }

    std::tuple<arma::mat, arma::mat, arma::mat> EKF::sigmas()
    {
        return std::make_tuple(sigma_prev, sigma_minus, sigma_plus);
    }

    void EKF::predict(Twist2D u)
    {
        int n = num_obstacles;
        arma::mat Q = arma::eye(3, 3);
        arma::mat Q_bar (3+(2*n), 3+(2*n), arma::fill::zeros);
        Q_bar.submat(0, 0, 2, 2) = Q;

        xi_minus = update_state(u);

        arma::mat A = calc_A(u);
        sigma_minus = (A*sigma_prev*A.t()) + Q_bar;
    }

    void EKF::update(int j, std::vector<double> z_sensor)
    {
        arma::mat I = arma::eye(3+(2*num_obstacles), 3+(2*num_obstacles));

        arma::mat H = calc_H(j);
        arma::mat R = arma::eye(2, 2);

        arma::vec z = z_sensor;
        arma::vec z_hat = calc_h(j);
        arma::mat K = sigma_minus*H.t()*((H*sigma_minus*H.t()) + R).t();
        xi_plus = xi_minus + K*(z - z_hat);
        sigma_plus = (I - (K*H))*sigma_minus;

        xi_prev = xi_plus;
        sigma_prev = sigma_plus;
    }


}