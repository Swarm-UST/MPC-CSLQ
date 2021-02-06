#pragma once

#include <Eigen/Dense>
#include <vector>
#include "constraint.h"
#include "plan_trajectory_potential.h"

void ilqr(Eigen::MatrixXd& X_opt, Eigen::MatrixXd& U_opt,std::vector<Eigen::MatrixXd>& K_opt, const Eigen::MatrixXd& X_src, const Eigen::MatrixXd& U_src, const Eigen::Vector3d& x_tar, double dt, double l, double r_wheel,const std::vector<trajectory_opt::Constraint> &soft_constraints, const std::vector<trajectory_opt::Constraint> &hard_constraints);

int MPC(Eigen::MatrixXd& X_MPC, Eigen::MatrixXd& U_MPC,std::vector<Eigen::MatrixXd>& K_MPC, Eigen::MatrixXd& X_all,const Eigen::Vector3d& x, const Eigen::Vector3d& x_tar, double T_MPC, double dt, double l, double r_wheel,const std::vector<trajectory_opt::Constraint> &soft_constraints, const std::vector<trajectory_opt::Constraint> &hard_constraints,double tol);