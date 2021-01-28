#include "trajectory_opt/utility.h"


constexpr double speed_limit = 8;

Eigen::Vector3d f(const Eigen::Vector3d &x, const Eigen::Vector2d &u, double dt, double l, double r)
{
    double theta = x(2);
    Eigen::Matrix<double, 3, 2> A;
    A << cos(theta), 0,
        sin(theta), 0,
        0, 1;
    Eigen::Matrix2d J;
    J << r / 2, r / 2,
        r / l, -r / l;
    Eigen::Vector3d x_next = x + dt * A * J * u;
    x_next(2) = atan2(sin(x_next(2)), cos(x_next(2)));
    return x_next;
}

Eigen::Matrix3d A_linear(const Eigen::Vector3d &x, const Eigen::Vector2d &u, double dt, double L, double R)
{
    double u1 = u(0), u2 = u(1), t = dt, x3 = x(2);

    Eigen::Matrix3d A;
    A << 1, 0, -(R * t * u1 * sin(x3)) / 2 - (R * t * u2 * sin(x3)) / 2,
        0, 1, (R * t * u1 * cos(x3)) / 2 + (R * t * u2 * cos(x3)) / 2,
        0, 0, 1;

    return A;
}

Eigen::Matrix<double, 3, 2> B_linear(const Eigen::Vector3d &x, const Eigen::Vector2d &u, double dt, double L, double R)
{
    double u1 = u(0), u2 = u(1), t = dt, x3 = x(2);

    Eigen::Matrix<double, 3, 2> B;
    B << (R * t * cos(x3)) / 2, (R * t * cos(x3)) / 2,
        (R * t * sin(x3)) / 2, (R * t * sin(x3)) / 2,
        (R * t) / L, -(R * t) / L;
    return B;
}

double cost(const Eigen::MatrixXd &X, const Eigen::MatrixXd &U, const Eigen::Matrix3d &Q, const Eigen::Matrix2d &R, const Eigen::Matrix3d &Qf, const Eigen::Vector3d &x_tar, const Eigen::Vector2d &u_tar)
{
    Eigen::MatrixXd state_diff = X.colwise() - x_tar;
    double l_state = (state_diff.leftCols(state_diff.cols() - 1).transpose() * Q * state_diff.leftCols(state_diff.cols() - 1)).trace();
    Eigen::MatrixXd input_diff = U.colwise() - u_tar;
    double l_u = (input_diff.transpose() * R * input_diff).trace();
    double l_f = (state_diff.rightCols(1).transpose() * Qf * state_diff.rightCols(1))(0, 0);
    return l_state + l_f + l_u;
}

Eigen::Vector2d twistToWheelSpeed(const Eigen::Vector2d twist, const double l, const double r)
{
    Eigen::Matrix2d J;
    J << r / 2, r / 2,
        r / l, -r / l;
    return J.inverse() * twist;
}

Eigen::Vector2d wheelSpeedToTwist(const Eigen::Vector2d speed, const double l, const double r)
{
    Eigen::Matrix2d J;
    J << r / 2, r / 2,
        r / l, -r / l;
    return J * speed;
}

void saturateWheelSpeed(Eigen::Vector2d &speed)
{
    if (std::abs(speed(0)) > (speed_limit-1) || std::abs(speed(1)) >  (speed_limit-1))
    {
        double ratio =  (speed_limit-1) / std::max<double>(std::abs(speed(0)), std::abs(speed(1)));
        speed = speed.eval() * ratio;
    }
}

double getUMax(void)
{
    return speed_limit;
}

void saturateTwist(Eigen::Vector2d &twist, const double l, const double r)
{
    Eigen::Vector2d speed = twistToWheelSpeed(twist, l, r);
    saturateWheelSpeed(speed);
    twist = wheelSpeedToTwist(speed, l, r);
}

bool isInConstraints(Eigen::Vector2d cur_pos, std::vector<trajectory_opt::Constraint> constraints)
{
    for (int i = 0; i < constraints.size(); i++)
    {
        if(constraints[i].isInConstraint(cur_pos)) return true;
    }
    return false;
}
