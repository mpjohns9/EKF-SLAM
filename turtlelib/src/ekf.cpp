#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"
#include <cmath>

/// \file 
/// \brief Extended Kalman Filter implementation

namespace turtlelib
{
    EKF::EKF()
    {
        state = {0.0, 0.0, 0.0};
        q = {state.ang, state.x, state.y};
    }

    EKF::EKF(Config s)
    {
        state = s;
        q = {state.ang, state.x, state.y};
    }

    EKF::EKF(Config s, std::vector<double> obs_x, std::vector<double> obs_y, int n)
    {
        state = s;
        q = {state.ang, state.x, state.y};
        num_obstacles = n;
        obstacles = arma::vec(2*n);
        int index = 0;
        for (int i=0; i<int(obs_x.size()); i++)
        {
            obstacles[index] = obs_x.at(i);
            obstacles[index+1] = obs_y.at(i);
            index += 2;
        }
        q = {state.ang, state.x, state.y};
        m = join_cols(q, obstacles);
    }

    arma::vec EKF::map_state()
    {
        return m;
    }

    // aram::mat EKF::get_h(int j)
    // {
    //     double mx = 
    //     double r = sqrt(pow(mx - x, 2))
    // }


}