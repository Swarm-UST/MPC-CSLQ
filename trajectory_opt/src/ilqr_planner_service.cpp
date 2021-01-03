#include "trajectory_opt/ilrqr_planner_service.h"

namespace trajectory_opt
{

    IlqrService::IlqrService(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
        optimize_trajectory_service_ = nh.advertiseService("optimize_trajectory", &IlqrService::optimize_trajectory, this);
    }

    void IlqrService::error(OptimizeTrajectory::Response &response, std::string msg)
    {
        response.success = false;
        response.error = msg;
    }

    bool IlqrService::optimize_trajectory(OptimizeTrajectory::Request &request, OptimizeTrajectory::Response &response)
    {
        if (request.state.layout.dim[1].size != 3)
        {
            error(response, "The size of state is wrong!");
        }
        else if (request.control_input.layout.dim[1].size != 2)
        {
            error(response, "The size of control_input is wrong!");
        }
        else if (request.Qf.layout.dim[1].size != 3 || request.Qf.layout.dim[0].size != 3)
        {
            error(response, "The size of Qf is wrong!");
        }
        else if (request.Q.layout.dim[1].size != 3 || request.Q.layout.dim[0].size != 3)
        {
            error(response, "The size of Q is wrong!");
        }
        else if (request.R.layout.dim[1].size != 2 || request.R.layout.dim[0].size != 2)
        {
            error(response, "The size of R is wrong!");
        }
        else
        {
            Eigen::MatrixXd Q, Qf, R, tar, X, U;
            msgTomatrixEigen(request.state, X);
            msgTomatrixEigen(request.control_input, U);
            msgTomatrixEigen(request.Q, Q);
            msgTomatrixEigen(request.Qf, Qf);
            msgTomatrixEigen(request.R, R);
            msgTomatrixEigen(request.target, tar);

            Eigen::Vector3d tar_v(tar(0, 0), tar(1, 0), tar(2, 0));
            IlqrPlanner planner(request.l, request.r, 100.0, 50, 20, Q, R, Qf);

            Eigen::MatrixXd X_opt, U_opt;
            planner.optimize_trajectory(X_opt, U_opt, X, U, tar_v, request.dt);

            response.success = true;
            matrixEigenToMsg(X_opt, response.opt_state);
            matrixEigenToMsg(U_opt, response.opt_input);
            return true;
        }
        return false;
    }
} // namespace trajectory_opt