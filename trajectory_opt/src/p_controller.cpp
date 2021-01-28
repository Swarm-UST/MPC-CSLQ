#include "trajectory_opt/p_controller.h"



Eigen::Vector2d p_controller(Eigen::Vector3d x, Eigen::Vector3d tar, double l, double r)
{
    double Kp_linear = 1500/(r);
    double Kp_angular = 3000/(r);

    Eigen::Vector3d error = tar - x;
    Eigen::Vector2d error_2d = error.head(2);
    Eigen::Vector3d heading_direction(cos(x(2)),sin(x(2)),0);
    //double norm_e = error_2d.norm()*pow((error/error_2d.norm()).dot(heading_direction),3.0);
    //double norm_e = error_2d.norm()*((error/error_2d.norm()).dot(heading_direction)>0?1:-1);
    double norm_e = error_2d.norm();
    double theta_e = atan2(error(1), error(0)) - x(2);

    while (theta_e > M_PI)
        theta_e = theta_e - 2*M_PI;
    while (theta_e < -M_PI)
        theta_e = theta_e + 2*M_PI;

    double v = Kp_linear * norm_e;
    double w = Kp_angular * theta_e;

    return Eigen::Vector2d(v, w);
}
