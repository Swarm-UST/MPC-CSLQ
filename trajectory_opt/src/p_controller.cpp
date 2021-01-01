#include "trajectory_opt/p_controller.h"

namespace trajectory_opt
{
    Eigen::Vector2d p_controller(Eigen::Vector3d x, Eigen::Vector3d tar, double l, double r)
    {
        double Kp_linear = 1.0;
        double Kp_angular = 3.14;

        Eigen::Vector3d error = tar - x;
        double norm_e = error.norm();
        double theta_e = atan2(error(1), error(0)) - x(2);

        while (theta_e > M_PI)
            theta_e = theta_e - 2 * M_PI;
        while (theta_e < -M_PI)
            theta_e = theta_e + 2 * M_PI;

        double v = Kp_linear * norm_e;
        double w = Kp_angular * theta_e;

        double v_up_limit = 2.0;

        if (v > v_up_limit)
            v = v_up_limit;
        else if (v < -v_up_limit)
            v = -v_up_limit;

        double w_up_limit = M_PI;
        if (w > w_up_limit)
            w = w_up_limit;
        else if (w < -w_up_limit)
            w = -w_up_limit;

        Eigen::Matrix2d J;
        J << r / 2, r / 2,
            r / l, -r / l;
        return J.inverse() * Eigen::Vector2d(v, w);
    }
} // namespace trajectory_opt