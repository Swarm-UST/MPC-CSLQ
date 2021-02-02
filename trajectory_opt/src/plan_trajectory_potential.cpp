#include "trajectory_opt/plan_trajectory_potential.h"
#include <iostream>
namespace trajectory_opt
{

    double gain = 0.0001;    
    double w_gain = 10;
    bool tra_count = false;
    void planTrajectory(Eigen::MatrixXd &X, Eigen::MatrixXd &U, const Eigen::Vector3d start_pos, const Eigen::Vector3d tar_pos, const std::vector<Constraint> &soft_constraints, const std::vector<Constraint> &hard_constraints, const double dt, const double l, const double r, const double tolerance)
    {
        if (tra_count == true)
        {
            std::cout<<"Please input the gain"<<std::endl;
            std::cin >> gain;
            std::cout<<"Please input the w_gain"<<std::endl;
            std::cin >> w_gain;
            tra_count = false;
        }

        Eigen::Vector3d cur_pos = start_pos;
        std::vector<double> X_container, U_container;
        X_container.reserve(500);
        U_container.reserve(500);
        X_container.push_back(cur_pos(0));
        X_container.push_back(cur_pos(1));
        X_container.push_back(cur_pos(2));
        Eigen::Vector2d error = cur_pos.head(2) - tar_pos.head(2);
        int count = 0;
        while (true)//error.norm() > tolerance)
        {
            count += 1;
            if (count > 2000) break;
            Eigen::Vector2d ori_twist = p_controller(cur_pos, tar_pos, l, r);
            Eigen::Vector2d cur_orien_vec(cos(cur_pos(2)), sin(cur_pos(2)));
            
            Eigen::Vector2d u = twistToWheelSpeed(ori_twist, l, r);
            saturateWheelSpeed(u);
            Eigen::Vector2d twist = wheelSpeedToTwist(u, l, r);
            Eigen::Vector2d cur_linear_velocity_vec = cur_orien_vec * twist(0);

            Eigen::Vector2d force(0, 0);
            bool isInSoftConstraint = false;
            for (int i = 0; i < soft_constraints.size(); i++)
            {
                Eigen::Vector2d displace_vec;
                if (soft_constraints[i].isInConstraint(cur_pos.head(2)))
                {
                    displace_vec = hard_constraints[i].getDisplacementFromBoundary(cur_pos.head(2));
                    isInSoftConstraint = true;
                    std::cout<<"In constraint"<<std::endl;
                }
                else
                {
                    displace_vec = soft_constraints[i].getDisplacementFromBoundary(cur_pos.head(2));
                }
                force += (gain / displace_vec.squaredNorm()) * displace_vec.normalized();
            }
            if (isInSoftConstraint == true)
            {
                force *= 100;
            }
                
            double force_in_linear_velocity = force.dot(cur_linear_velocity_vec.normalized());
            Eigen::Vector2d force_in_linear_velocity_vec = force_in_linear_velocity * cur_linear_velocity_vec.normalized();
            if (isInSoftConstraint)
            {
                twist(0) += force_in_linear_velocity * static_cast<double>(get_sign<double>(twist(0)));
                Eigen::Vector3d rotation_vec = Eigen::Vector3d(cur_linear_velocity_vec(0), cur_linear_velocity_vec(1), 0).cross(Eigen::Vector3d(force(0), force(1), 0));
                if (rotation_vec(2) > 0)
                {
                    twist(1) += 10*w_gain * (force - force_in_linear_velocity_vec).norm();
                }
                else
                {
                    twist(1) -= 10*w_gain * (force - force_in_linear_velocity_vec).norm();
                }
            }
            else
            {
                // twist(0) += (force_in_linear_velocity < 0 ? force_in_linear_velocity:0)* static_cast<double>(get_sign<double>(twist(0))) ;
                double force_in_linear_velocity_zero = (force_in_linear_velocity < 0 ? force_in_linear_velocity : 0);
                twist(0) += (force_in_linear_velocity_zero < -abs(twist(0)) ? -abs(twist(0)):force_in_linear_velocity_zero)* static_cast<double>(get_sign<double>(twist(0))) ;
                Eigen::Vector3d rotation_vec = Eigen::Vector3d(cur_linear_velocity_vec(0), cur_linear_velocity_vec(1), 0).cross(Eigen::Vector3d(force(0), force(1), 0));
                if (rotation_vec(2) > 0)
                {
                    twist(1) += w_gain * std::abs(force_in_linear_velocity_zero);
                }
                else
                {
                    twist(1) -= w_gain *  std::abs(force_in_linear_velocity_zero);
                }
            }

            u = twistToWheelSpeed(twist, l, r);
            saturateWheelSpeed(u);
            U_container.push_back(u(0));
            U_container.push_back(u(1));
            cur_pos = f(cur_pos, u, dt, l, r);
            X_container.push_back(cur_pos(0));
            X_container.push_back(cur_pos(1));
            X_container.push_back(cur_pos(2));
            error = cur_pos.head(2) - tar_pos.head(2);
            //std::cout<<error.norm()<<std::endl;
        }
        Eigen::Map<Eigen::MatrixXd> X_m(X_container.data(), 3, (int)(X_container.size() / 3));
        Eigen::Map<Eigen::MatrixXd> U_m(U_container.data(), 2, (int)(U_container.size() / 2));
        X = X_m;
        U = U_m;
    }
} // namespace trajectory_opt
