
#include "trajectory_opt/init_trajectory.h"

namespace trajectory_opt
{
    void init_trajectory(Eigen::MatrixXd &X, Eigen::MatrixXd &U, const Eigen::MatrixXd waypoints, const double dt, const double l, const double r, const double thres)
    {
        std::vector<double> x_vec, u_vec;
        x_vec.push_back(waypoints(0, 0));
        x_vec.push_back(waypoints(1, 0));
        x_vec.push_back(waypoints(2, 0));

        for (int i = 1; i < waypoints.cols(); i++)
        {
            Eigen::Vector3d x_tar = waypoints.col(i);
            while (true)
            {
                Eigen::Vector3d x_start(x_vec.end()[-3], x_vec.end()[-2], x_vec.end()[-1]);
                Eigen::Vector2d u_k = p_controller(x_start, x_tar, l, r);
                u_vec.push_back(u_k(0));
                u_vec.push_back(u_k(1));
                Eigen::Vector3d x_k = f(x_start, u_k, dt, l, r);
                x_vec.push_back(x_k(0));
                x_vec.push_back(x_k(1));
                x_vec.push_back(x_k(2));

                if ((x_tar - x_k).head(2).norm() < thres)
                    break;
            }
        }
        Eigen::Map<Eigen::MatrixXd> X_m(x_vec.data(), 3, (int)(x_vec.size() / 3));
        Eigen::Map<Eigen::MatrixXd> U_m(u_vec.data(), 2, (int)(u_vec.size() / 2));
        X = X_m;
        U = U_m;
    }
} // namespace trajectory_opt