#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"
#include <armadillo>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
// #include <tuple>

/// \file 
/// \brief Extended Kalman Filter implementation

namespace turtlelib
{

    EKF::EKF(int n)
    {
        state = {0.0, 0.0, 0.0};
        q = {state.ang, state.x, state.y};
        num_obstacles = n;
        obstacles = arma::vec(2*n, arma::fill::zeros);
        xi_prev = arma::vec(3 + (2*n), arma::fill::zeros);
        xi_minus = arma::vec(3 + (2*n), arma::fill::zeros);
        xi_plus = arma::vec(3 + (2*n), arma::fill::zeros);
        sigma_minus = arma::mat(3+(2*n), 3+(2*n), arma::fill::zeros);
        arma::mat sig_m = arma::eye(2*n, 2*n);
        sig_m = sig_m*INT_MAX;
        sigma_minus.submat(3, 3, 3+(2*n)-1, 3+(2*n)-1) = sig_m;
        sigma_prev = sigma_minus;
        // sigma_prev.print("SIGMA PREV");
        sigma_plus = sigma_minus;
    }

    EKF::EKF(Config s, int n)
    {
        state = s;
        q = {state.ang, state.x, state.y};
        num_obstacles = n;
        obstacles = arma::vec(2*n);
        xi_prev = arma::vec(3 + (2*n), arma::fill::zeros);
        xi_plus = xi_prev;

        sigma_minus = arma::mat(3+(2*n), 3+(2*n), arma::fill::zeros);
        arma::mat sig_m = arma::eye(2*n, 2*n);
        sig_m = sig_m*INT_MAX;
        sigma_minus.submat(3, 3, 3+(2*n)-1, 3+(2*n)-1) = sig_m;

        sigma_prev = sigma_minus;
    }

    arma::vec EKF::initialize_landmarks(std::vector<double> obs_x, std::vector<double> obs_y)
    {
        // ROS_ERROR_STREAM("INIT LANDMARKS");
        int index = 0;
        for (int i=0; i<int(obs_x.size()); i++)
        {
            // ROS_ERROR_STREAM("SIZE OF OBS VEC: " << obs_x.size());
            // ROS_ERROR_STREAM("LAND X: " << obs_x.at(i));
            // ROS_ERROR_STREAM("LAND Y: " << obs_y.at(i));
            double r = sqrt(pow(obs_x.at(i), 2) + pow(obs_y.at(i), 2));
            // ROS_ERROR_STREAM("r: " << r);
            double phi = atan2(obs_y.at(i), obs_x.at(i));
            // ROS_ERROR_STREAM("phi: " << r);

            double mx = xi_prev.at(1) + r*cos(phi + xi_prev.at(0));
            // ROS_ERROR_STREAM("mx: " << r);
            double my = xi_prev.at(2) + r*sin(phi + xi_prev.at(0));
            // ROS_ERROR_STREAM("my: " << r);

            Vector2D v;
            v.x = mx;
            v.y = my;

            // Vector2D mb;
            // mb.x = xi_prev.at(1);
            // mb.y = xi_prev.at(2);

            // Transform2D Tmb(mb);

            std::vector<Vector2D> known_obs_copy = known_obs;
            bool exists = false;

            // ROS_ERROR_STREAM("SIZE: " << known_obs_copy.size());
            if (known_obs_copy.size() > 0)
            {
                for (int j=0; j<int(known_obs_copy.size()); j++)
                {   
                    
                    // ROS_ERROR_STREAM("CHECKING KNOWN OBSTACLE #" << j+1);

                    // ROS_ERROR_STREAM("KNOWN X: " << known_obs.at(j).x);
                    // ROS_ERROR_STREAM("CURRENT X: " << obs_x.at(i));

                    // ROS_ERROR_STREAM("KNOWN Y: " << known_obs.at(j).y);
                    // ROS_ERROR_STREAM("CURRENT Y: " << obs_y.at(i));

                    if ((abs(known_obs.at(j).x - mx) < 0.1) && (abs(known_obs.at(j).y - my) < 0.1))
                    {
                        exists = true;
                        // ROS_ERROR_STREAM("OBSTACLE EXISTS ALREADY");
                        break;
                    }
                }

                if (!exists)
                {
                    // ROS_ERROR_STREAM_ONCE("PUSHING BACK -- NEW OBS");
                    known_obs.push_back(v);
                    obstacles.at(index) = mx;
                    obstacles.at(index+1) = my;
                    index += 2;
                }
            }
            else
            {
                // ROS_ERROR_STREAM_ONCE("PUSHING BACK -- FIRST OBS");
                known_obs.push_back(v);
                obstacles.at(index) = mx;
                obstacles.at(index+1) = my;
                index += 2;
            }
            // ROS_ERROR_STREAM("OBS SIZE: " << known_obs.size());
            // ROS_ERROR_STREAM("______________________________________________________________________");
        }

        // ROS_ERROR_STREAM("XI: " << xi_prev);
        xi_prev = arma::join_cols(q, obstacles);
        // ROS_ERROR_STREAM("XI PREV INIT: " << xi_prev);
        return xi_prev;
    }

    arma::vec EKF::calc_h(int j)
    {
        // ROS_ERROR_STREAM("CALC h");
        double mx = xi_minus.at(3+(2*j));
        double my = xi_minus.at(4+(2*j));
        double r = sqrt(pow(mx - xi_minus.at(1), 2) + pow(my - xi_minus.at(2), 2));
        double phi = atan2(my - xi_minus.at(2), mx - xi_minus.at(1)) - xi_minus.at(0);

        arma::vec h = {r, phi};
        return h;
    }

    arma::mat EKF::calc_H(int j)
    {
        // ROS_ERROR_STREAM("CALC H");
        double mx = xi_minus.at(3+(2*j));
        double my = xi_minus.at(4+(2*j));
        // ROS_ERROR_STREAM("MX: " << mx);
        // ROS_ERROR_STREAM("MY: " << my);

        double dx = mx - xi_minus.at(1);
        double dy = my - xi_minus.at(2);

        double d = pow(dx, 2) + pow(dy, 2);
        // ROS_ERROR_STREAM("d: " << d);

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
        H.submat(0, 3 + (2*j), 1, 4 + (2*j)) = h_m;

        // H.print("H");

        return H;
    }

    arma::mat EKF::calc_A(Twist2D u)
    {
        // ROS_ERROR_STREAM("CALC A");
        arma::mat I = arma::eye((3+(2*num_obstacles)), (3+(2*num_obstacles)));
        arma::mat mat((3+(2*num_obstacles)), (3+(2*num_obstacles)), arma::fill::zeros);

        // ROS_ERROR_STREAM("TWIST: " << u);

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

        // mat.print("mat");

        arma::mat A = I + mat;
        return A;
    }

    arma::vec EKF::update_state(Twist2D u)
    {
        // ROS_ERROR_STREAM("UPDATE");
        if (almost_equal(u.ang, 0.0))
        {
            arma::vec ut(3+(2*num_obstacles), arma::fill::zeros);
            ut.at(1) = u.x*cos(xi_prev.at(0));
            ut.at(2) = u.x*sin(xi_prev.at(0));
            // ROS_ERROR_STREAM("Ut: " << ut);

            xi_minus = xi_prev + ut;
            // ROS_ERROR_STREAM("U = 0: " << xi_minus);
            return xi_minus;
        }
        else
        {
            arma::vec ut(3+(2*num_obstacles), arma::fill::zeros);
            ut.at(0) = u.ang;
            ut.at(1) = ((-u.x/u.ang)*sin(xi_prev.at(0))) + ((u.x/u.ang)*sin(xi_prev.at(0) + u.ang));
            ut.at(2) = ((-u.x/u.ang)*cos(xi_prev.at(0))) + ((u.x/u.ang)*cos(xi_prev.at(0) + u.ang));

            xi_minus = xi_prev + ut;
            // ROS_ERROR_STREAM("U != 0: " << xi_minus);
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

    Config EKF::config()
    {
        return Config{xi_plus.at(0), xi_plus.at(1), xi_plus.at(2)};
    }

    std::vector<double> EKF::obs_vec()
    {
        std::vector<double> v;
        v = arma::conv_to < std::vector<double> >::from(xi_plus);
        std::vector<double> v_out = std::vector<double>(v.begin()+3, v.end());

        // for (int i=0; i<int(v_out.size()); i+=2)
        // {
        //     auto curr = v_out.at(i);
        //     auto next = v_out.at(i+1);
        //     if (curr == 0.0 && next == 0.0)
        //     {
        //         v_out = std::vector<double>(v_out.begin()+3, v_out.begin()+(i-1));
        //         break;
        //     }
            
        // }
        return v_out;
    }

    void EKF::predict(Twist2D u)
    {
        // ROS_ERROR_STREAM("PREDICT");
        int n = num_obstacles;
        arma::mat Q = arma::eye(3, 3);
        arma::mat Q_bar(3+(2*n), 3+(2*n), arma::fill::zeros);
        Q_bar.submat(0, 0, 2, 2) = Q;
        // Q_bar.print("Q");

        xi_minus = update_state(u);
        // xi_minus.print("XI MINUS");

        arma::mat A = calc_A(u);
        // A.print("A");
        sigma_minus = (A*sigma_prev*A.t()) + Q_bar;
    }

    void EKF::update(int j, double x, double y)
    {
        // ROS_ERROR_STREAM("UPDATE");
        // xi_plus.print("XI INIT");
        arma::mat I = arma::eye(3+(2*num_obstacles), 3+(2*num_obstacles));

        arma::mat H = calc_H(j);
        arma::mat R = arma::eye(2, 2);

        // ROS_ERROR_STREAM("X: " << x);
        // ROS_ERROR_STREAM("Y: " << y);
        double r = sqrt(pow(x, 2) + pow(y, 2));
        double phi = normalize_angle(atan2(y, x));

        arma::vec z = {r, phi};
        // z.print("Z");

        arma::vec z_hat = calc_h(j);
        z_hat.at(1) = normalize_angle(z_hat.at(1));
        // z_hat.print("Z HAT");
        // R.print("R");
        // H.print("H");
        // sigma_minus.print("SIG MIN");
        arma::mat K = (sigma_minus*H.t())*(((H*sigma_minus*H.t()) + R).i());
        // K.print("K"); 
        arma::vec z_diff = z - z_hat;
        // z_diff.print("Z DIFF");
        z_diff.at(1) = normalize_angle(z_diff.at(1));
        // z_diff.print("Z DIFF NORMAL");
        
        xi_plus = xi_minus + K*(z_diff);
        sigma_plus = (I - (K*H))*sigma_minus;

        xi_prev = xi_plus;
        // xi_prev.print("XI PREV");
        sigma_prev = sigma_plus;
        // xi_plus.print("XI");
    }

    bool EKF::check_known_obs(double x, double y)
    {
        // Vector2D v;
        // v.x = x;
        // v.y = y;

        // Vector2D mb;
        // mb.x = xi_prev.at(1);
        // mb.y = xi_prev.at(2);

        // Transform2D Tmb(mb);

        // Vector2D v_mb = Tmb(v);

        double r = sqrt(pow(x, 2) + pow(y, 2));
        // ROS_ERROR_STREAM("r: " << r);
        double phi = atan2(y, x);
        // ROS_ERROR_STREAM("phi: " << r);

        double mx = xi_prev.at(1) + r*cos(phi + xi_prev.at(0));
        // ROS_ERROR_STREAM("mx: " << r);
        double my = xi_prev.at(2) + r*sin(phi + xi_prev.at(0));
        // ROS_ERROR_STREAM("my: " << r);

        Vector2D v;
        v.x = mx;
        v.y = my;

        for (int i=0; i<int(known_obs.size()); i++)
        {
            if (abs(known_obs.at(i).x - mx) > 0.1 && abs(known_obs.at(i).y - my) > 0.1)
            {
                return true;
            }
        }
        return false;
    }


}