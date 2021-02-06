#include "trajectory_opt/ilrqr_planner_service.h"

namespace trajectory_opt
{

    IlqrService::IlqrService(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
        optimize_trajectory_service_ = nh.advertiseService("optimize_trajectory", &IlqrService::optimize_trajectory, this);
        global_path_pub_ = nh.advertise<nav_msgs::Path>("global_trajectory", 1);
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
        else if (request.target.layout.dim[1].size != 3)
        {
            error(response, "The size of target is wrong!");
        
        }
        else if (request.constraints.layout.dim[1].size != 6)
        {
            error(response, "The size of constraints is wrong!");
        }
        else
        {
            Eigen::MatrixXd init,tar,Cons;
            msgTomatrixEigen(request.state, init);
            
            msgTomatrixEigen(request.target, tar);
            msgTomatrixEigen(request.constraints,Cons);
            double r = 0.025, l = 0.0835, dt = 1.0/120;



            std::vector<trajectory_opt::Constraint> soft_constraints, hard_constraints;
            for(int i = 0; i < Cons.cols(); i++)
            {
                if (Cons(5,i) == 0) // If the constraint type is circle
                {
                    soft_constraints.emplace_back(Eigen::Vector2d(Cons(0,i),Cons(1,i)),Cons(2,i)+Cons(4,i));
                    hard_constraints.emplace_back(Eigen::Vector2d(Cons(0,i),Cons(1,i)),Cons(2,i));

                }
                else // If the constraint type is retangular
                {
                    hard_constraints.emplace_back(Eigen::Vector2d(Cons(0,i),Cons(1,i)),Cons(2,i),Cons(3,i));
                    soft_constraints.emplace_back(Eigen::Vector2d(Cons(0,i)-Cons(4,i),Cons(1,i)-Cons(4,i)),Cons(2,i)+2*Cons(4,i),Cons(3,i)+2*Cons(4,i));
                }
            }
            double T_Horizon = request.T_horizon;

            Eigen::Vector3d X_tar(tar(0, 0), tar(1, 0), tar(2, 0));
            Eigen::Vector3d X_init(init(0,0), init(1,0), init(2,0));

            Eigen::MatrixXd X_MPC,U_MPC, X_all;
            std::vector<Eigen::MatrixXd> K_MPC;
            MPC(X_MPC,U_MPC, K_MPC, X_all,X_init, X_tar, T_Horizon, dt, l, r, soft_constraints, hard_constraints, 0.01);
            nav_msgs::Path global_path;

            constexpr int skip_num = 30;
            Eigen::Map<Eigen::MatrixXd,0,Eigen::OuterStride<> > X_reduced(X_all.data(), X_all.rows(), (X_all.cols()+(skip_num - 1))/skip_num, Eigen::OuterStride<>(X_all.outerStride()*skip_num));
            stateToPath(X_reduced.leftCols(20), global_path);
            global_path_pub_.publish(global_path);

            response.success = true;
            matrixEigenToMsg(X_MPC, response.opt_state);
            matrixEigenToMsg(U_MPC, response.opt_input);
            VectorEigenMatrixToMsg(K_MPC,response.opt_feedback);
            return true;
        }
        return false;
    }
} // namespace trajectory_opt