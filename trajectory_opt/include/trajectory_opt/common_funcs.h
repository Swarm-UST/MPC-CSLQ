#ifndef COMMON_FUNCS_H
#define COMMON_FUNCS_H
#include <Eigen/Dense>
#include <cmath>
namespace trajectory_opt
{
    Eigen::Vector3d f(const Eigen::Vector3d &x, const Eigen::Vector2d &u, double dt, double l, double r);
    Eigen::Matrix3d A_linear(const Eigen::Vector3d &x, const Eigen::Vector2d &u, double dt, double L, double R);
    Eigen::Matrix<double, 3, 2> B_linear(const Eigen::Vector3d &x, const Eigen::Vector2d &u, double dt, double L, double R);
    double cost(const Eigen::MatrixXd &X, const Eigen::MatrixXd &U, const Eigen::Matrix3d &Q, const Eigen::Matrix2d &R, const Eigen::Matrix3d &Qf, const Eigen::Vector3d &x_tar, const Eigen::Vector2d &u_tar);
} // namespace trajectory_opt
#endif