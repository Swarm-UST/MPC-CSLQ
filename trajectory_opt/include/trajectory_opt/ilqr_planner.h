#ifndef ILQR_PLANNER_H
#define ILQR_PLANNER_H

#include <ros/ros.h>
#include "trajectory_opt/common_funcs.h"
#include <Eigen/Dense>

namespace trajectory_opt
{
    class IlqrPlanner
    {
    private:
        double r_wheel_;
        double length_;
        double cost_thres_;
        int iter_num_;
        int line_iter_num_;
        Eigen::Matrix3d Q;
        Eigen::Matrix2d R;
        Eigen::Matrix3d Qf;

    public:
        IlqrPlanner(double l, double r, double cost_thres, int iter_num, int line_iter_num, Eigen::Matrix3d Q, Eigen::Matrix2d R, Eigen::Matrix3d Qf);
        bool optimize_trajectory(Eigen::MatrixXd &X_opt, Eigen::MatrixXd &U_opt, Eigen::MatrixXd X, Eigen::MatrixXd U, const Eigen::Vector3d x_tar,const double dt) const;
    };
} // namespace trajectory_opt

#endif