#include "trajectory_opt/ilqr_planner.h"

namespace trajectory_opt
{
    IlqrPlanner::IlqrPlanner(double l, double r, double cost_thres, int iter_num, int line_iter_num, Eigen::Matrix3d Q, Eigen::Matrix2d R, Eigen::Matrix3d Qf) : length_(l), r_wheel_(r), cost_thres_(cost_thres), iter_num_(iter_num), line_iter_num_(line_iter_num)
    {
        this->Q = Q;
        this->Qf = Qf;
        this->R = R;
    }
    bool IlqrPlanner::optimize_trajectory(Eigen::MatrixXd &X_opt, Eigen::MatrixXd &U_opt, Eigen::MatrixXd X, Eigen::MatrixXd U, const Eigen::Vector3d x_tar, const double dt) const
    {
        ROS_DEBUG_STREAM("Q \n"
                         << Q << "\nQf \n"
                         << Qf << "\nR \n"
                         << R << '\n');
        ROS_DEBUG_STREAM("l " << length_ << " r " << r_wheel_ << " cost_thres_ " << cost_thres_ << '\n');
        ROS_DEBUG_STREAM(" iter_num_ " << iter_num_ << " line_iter_num_ " << line_iter_num_ << '\n');

        const int N = X.cols();

        std::vector<Eigen::MatrixXd> K_(N - 1, Eigen::MatrixXd::Zero(2, 3));
        Eigen::MatrixXd l_(2, N - 1);
        l_ << Eigen::MatrixXd::Zero(2, N - 1);
        Eigen::MatrixXd U_ = Eigen::MatrixXd::Zero(2, N - 1);
        Eigen::MatrixXd X_ = Eigen::MatrixXd::Zero(3, N);
        X_.col(0) = X.col(0);

        for (int j = 0; j < iter_num_; j++)
        {
            double c_init = cost(X, U, Q, R, Qf, x_tar, Eigen::Vector2d::Zero());
            Eigen::Matrix3d cap_P(Qf);
            Eigen::Vector3d little_p;
            little_p = Qf * (X.rightCols(1) - x_tar);

            for (int n = N - 2; n >= 0; n--)
            {
                Eigen::Vector3d x = X.col(n);
                Eigen::Vector2d u = U.col(n);
                Eigen::Matrix3d A = A_linear(x, u, dt, length_, r_wheel_);
                Eigen::Matrix<double, 3, 2> B = B_linear(x, u, dt, length_, r_wheel_);
                Eigen::Vector3d q = Q * (x - x_tar);
                Eigen::Vector2d r = R * u;

                Eigen::MatrixXd G = B.transpose() * cap_P * A;
                Eigen::MatrixXd H = R + B.transpose() * cap_P * B;
                Eigen::MatrixXd g = r + B.transpose() * little_p;

                Eigen::MatrixXd K = -H.inverse() * G;
                K_[n] = K;
                l_.col(n) = -H.inverse() * g;
                cap_P = Q + A.transpose() * cap_P * A + K.transpose() * H * K + K.transpose() * G + G.transpose() * K;
                little_p = q + A.transpose() * little_p + K.transpose() * H * l_.col(n) + K.transpose() * g + G.transpose() * l_.col(n);
            }

            double scale = 2;
            double alpha = 1;
            for (int k = 0; k < line_iter_num_; k++)
            {
                for (int i = 0; i < N - 1; i++)
                {
                    U_.col(i) = U.col(i) + alpha * l_.col(i) + K_[i] * (X_.col(i) - X.col(i));
                    X_.col(i + 1) = f(X_.col(i), U_.col(i), dt, length_, r_wheel_);
                }
                alpha = alpha / scale;
                double c = cost(X_, U_, Q, R, Qf, x_tar, Eigen::Vector2d::Zero());
                if (c < c_init)
                {
                    break;
                }
            }
            U = U_;
            X = X_;
        }
        X_opt = X;
        U_opt = U;
    }

} // namespace trajectory_opt