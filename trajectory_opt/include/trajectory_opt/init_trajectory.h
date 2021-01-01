#ifndef INIT_TRAJECTORY_H
#define INIT_TRAJECTORY_H

#include "trajectory_opt/p_controller.h"
#include "trajectory_opt/common_funcs.h"
#include <vector>
namespace trajectory_opt
{
    void init_trajectory(Eigen::MatrixXd & X, Eigen::MatrixXd & U, const Eigen::MatrixXd waypoints, const double dt, const double l, const double r, const double thres);
}
#endif