#include <cmath>
#include <vector>
#include "trajectory_opt/p_controller.h"
#include "trajectory_opt/utility.h"
#include "trajectory_opt/constraint.h"
#include <iostream>

namespace trajectory_opt
{
    void planTrajectory(Eigen::MatrixXd &X, Eigen::MatrixXd &U, const Eigen::Vector3d start_pos, const Eigen::Vector3d tar_pos, const std::vector<Constraint> &soft_constraints, const std::vector<Constraint> &hard_constraints, const double dt, const double l, const double r, const double tolerance);
}
