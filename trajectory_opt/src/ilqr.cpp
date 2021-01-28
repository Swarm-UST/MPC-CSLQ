#include <Eigen/Dense>
#include <iostream>
#include "trajectory_opt/ilqr.h"
#include "trajectory_opt/utility.h"
#include <chrono>

void ilqr(Eigen::MatrixXd& X_opt, Eigen::MatrixXd& U_opt,std::vector<Eigen::MatrixXd>& K_opt, const Eigen::MatrixXd& X_src, const Eigen::MatrixXd& U_src, const Eigen::Vector3d& x_tar, double dt, double l, double r_wheel,const std::vector<trajectory_opt::Constraint> &soft_constraints, const std::vector<trajectory_opt::Constraint> &hard_constraints)
{
    Eigen::MatrixXd X = X_src, U = U_src;
    const int N = X.cols();

    
    Eigen::Matrix3d Q;
    Q <<          10, 0, 0,
                  0, 10, 0,
                  0, 0, 0;
    Eigen::Matrix2d R = 0.05 * Eigen::Matrix2d::Identity();
    Eigen::Matrix3d Qf;
    Qf <<          100000, 0, 0,
                  0, 100000, 0,
                  0, 0,     1;



    const int iter_num = 25;
    const int line_iter_num = 10;
    const int u_tar = 0;

    std::vector<Eigen::MatrixXd> K_(N-1, Eigen::MatrixXd::Zero(2,3));
    Eigen::MatrixXd l_(2,N-1);
    l_ << Eigen::MatrixXd::Zero(2, N-1);
    Eigen::MatrixXd U_ = Eigen::MatrixXd::Zero(2, N-1);
    Eigen::MatrixXd X_ = Eigen::MatrixXd::Zero(3, N);
    X_.col(0) = X.col(0);
    

    // determine whether the inital point is inside the soft_constraint
    bool start_isInSoftConstraint = false;
    for(int con_count = 0;con_count<soft_constraints.size();con_count++)
    {
        start_isInSoftConstraint|= soft_constraints[con_count].isInConstraint(X.col(0).head(2));
    }
    // If it start inside the soft_constraint, count how many time stamp is inside soft_constaint and will be used in the line search
    // If time stamp < t_inSoftConstraint, the algorithm will used the hard constraint instead of soft constraint
    // Else, Soft constraints 
    int t_inSoftConstraint = 0;
    if(start_isInSoftConstraint)
    {
        bool still_inSoftConstraint = true;
        while(still_inSoftConstraint)
        {
            t_inSoftConstraint++;
            bool isInSoftConstraint = false;
            for(int con_count = 0;con_count<soft_constraints.size();con_count++)
            {
                isInSoftConstraint|= soft_constraints[con_count].isInConstraint(X.col(t_inSoftConstraint).head(2));
            }
            still_inSoftConstraint &= isInSoftConstraint;
        }      
    }
   
    double c_init = cost(X, U, Q, R, Qf, x_tar, Eigen::Vector2d::Zero());
    for(int j = 0; j < iter_num;j++)
    {
        
        double c = 0;
        Eigen::Matrix3d cap_P(Qf);
        Eigen::Vector3d little_p;
        little_p = Qf * (X.rightCols(1) - x_tar);

        for(int n=N-2;n >= 0;n--)
        {
            Eigen::Vector3d x = X.col(n);
            Eigen::Vector2d u = U.col(n);
            Eigen::Matrix3d A = A_linear(x, u, dt, l, r_wheel);
            Eigen::Matrix<double, 3, 2> B = B_linear(x, u, dt, l, r_wheel);
            Eigen::Vector3d q = Q * (x-x_tar);
            Eigen::Vector2d r = R * u;

            Eigen::MatrixXd G = B.transpose() * cap_P * A;
            Eigen::MatrixXd H = R + B.transpose()*cap_P*B;
            Eigen::MatrixXd g = r + B.transpose() * little_p;

            Eigen::MatrixXd K = - H.inverse() * G;
            K_[n] = K;
            l_.col(n) = - H.inverse() * g;
            cap_P = Q + A.transpose()*cap_P*A + K.transpose()*H*K + K.transpose()*G+G.transpose()*K;
            little_p = q + A.transpose()*little_p+K.transpose()*H*l_.col(n)+K.transpose()*g+G.transpose()*l_.col(n);
        }

        double scale = 2;
        double alpha = 1;
        for(int k = 0; k < line_iter_num;k++)
        {
            bool isInConstraint = false;
            bool isOutUMax = false;
            if (k == line_iter_num-1){alpha = 0;}
            // This part is to check whether the trajectory has violated the constraint
            for(int i = 0;i < N-1;i++)
            {
                U_.col(i) = U.col(i) + alpha*l_.col(i) + K_[i]*(X_.col(i) - X.col(i));
                X_.col(i+1) = f(X_.col(i),U_.col(i),dt, l,r_wheel);
                // The number of soft constraint and hard constraint are assumed to be equal
                for(int con_count=0;con_count<soft_constraints.size();con_count++)
                {
                    // If it originally is in soft constraint, we will use hard constriant instead
                    if (i < t_inSoftConstraint)
                    {
                        isInConstraint|= hard_constraints[con_count].isInConstraint(X_.col(i+1).head(2));
                    }
                    // Else use the soft constraint
                    else
                    {
                        isInConstraint|= soft_constraints[con_count].isInConstraint(X_.col(i+1).head(2));
                    }
                    
                }
            }
            // Check whether the control input exceed the limit
            isOutUMax |= ((U_.maxCoeff()>getUMax()) || abs(U_.minCoeff())>getUMax());
            alpha = alpha/scale;
            c = cost(X_, U_, Q, R, Qf, x_tar, Eigen::Vector2d::Zero());
            if(c < c_init && !isInConstraint && !isOutUMax) {break;}
        }
        U = U_;
        X = X_;
        if (abs((c-c_init)/c_init)<0.1){break;}
        c_init = c;
    }
    X_opt = X;
    U_opt = U;
    K_opt = K_;
}



int MPC(Eigen::MatrixXd& X_MPC, Eigen::MatrixXd& U_MPC ,std::vector<Eigen::MatrixXd>& K_MPC, const Eigen::Vector3d& x, const Eigen::Vector3d& x_tar, double T_Horizon, double dt, double l, double r_wheel,const std::vector<trajectory_opt::Constraint> &soft_constraints, const std::vector<trajectory_opt::Constraint> &hard_constraints,double tol)
{
    auto start = std::chrono::high_resolution_clock::now(); 
    Eigen::MatrixXd X, U;
    trajectory_opt::planTrajectory(X, U, x, x_tar, soft_constraints, hard_constraints, dt, l, r_wheel, tol);

    int T = (X.cols() > int(T_Horizon/dt)?int(T_Horizon/dt):X.cols());
    
    X.conservativeResize(Eigen::NoChange,T);
    U.conservativeResize(Eigen::NoChange,T);

    //std::cout<<T<<std::endl;

    //X_MPC = X;
    //U_MPC = U;
    ilqr(X_MPC, U_MPC, K_MPC, X, U, x_tar, dt, l, r_wheel,soft_constraints, hard_constraints);
    auto stop = std::chrono::high_resolution_clock::now(); 
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start); 
    std::cout<<"Time for each iteration : "<<(float)(duration.count())/1000<<std::endl;
    return T;
}




