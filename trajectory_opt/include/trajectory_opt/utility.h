#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include "trajectory_opt/constraint.h"
#include <vector>

Eigen::Vector3d f(const Eigen::Vector3d &x, const Eigen::Vector2d &u, double dt, double l, double r);
Eigen::Matrix3d A_linear(const Eigen::Vector3d &x, const Eigen::Vector2d &u, double dt, double L, double R);
Eigen::Matrix<double, 3, 2> B_linear(const Eigen::Vector3d &x, const Eigen::Vector2d &u, double dt, double L, double R);
double cost(const Eigen::MatrixXd &X, const Eigen::MatrixXd &U, const Eigen::Matrix3d &Q, const Eigen::Matrix2d &R, const Eigen::Matrix3d &Qf, const Eigen::Vector3d &x_tar, const Eigen::Vector2d &u_tar);
Eigen::Vector2d twistToWheelSpeed(const Eigen::Vector2d twist, const double l, const double r);
Eigen::Vector2d wheelSpeedToTwist(const Eigen::Vector2d speed, const double l, const double r);
void saturateWheelSpeed(Eigen::Vector2d &speed);
void saturateTwist(Eigen::Vector2d &twist, const double l, const double r);
bool isInConstraints(Eigen::Vector2d cur_pos, std::vector<trajectory_opt::Constraint> constraints);
double getUMax(void);

template <typename T> int get_sign(T val) {
    return (T(0) < val) - (val < T(0));
}
