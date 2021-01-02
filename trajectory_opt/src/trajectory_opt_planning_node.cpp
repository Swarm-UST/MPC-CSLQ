#include <ros/ros.h>
#include "trajectory_opt/ilqr_planner.h"
#include "trajectory_opt/ros_conversion.h"
#include "trajectory_opt/init_trajectory.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ROS_INFO("Node start\n");

    double r = 0.025, l = 0.1, dt = 0.01;
    Eigen::Matrix3d Q;
    Q << 10, 0, 0,
        0, 10, 0,
        0, 0, 0;

    Eigen::Matrix2d R = 0.005 * Eigen::Matrix2d::Identity();

    Eigen::Matrix3d Qf;
    Qf << 100000, 0, 0,
        0, 100000, 0,
        0, 0, 1;
    Eigen::MatrixXd waypoints(3,6);
    waypoints << 0, 1, 2, 3, 4, 6,
                 0, 1, 1, -1, -1, 0,
                 0, 0, 0, 0, 0, 0;
    Eigen::MatrixXd X, U;
    trajectory_opt::init_trajectory(X, U, waypoints, dt, l, r, 0.1);
    ROS_INFO_STREAM("# of points " << X.cols() << "  # of U " << U.cols()<<'\n');

    trajectory_opt::IlqrPlanner planner(l, r, 100.0, 50, 20, Q, R, Qf);

    Eigen::MatrixXd X_opt, U_opt;

    ros::WallTime begin = ros::WallTime::now();
    planner.optimize_trajectory(X_opt, U_opt, X, U, waypoints.rightCols(1), dt);
    ROS_INFO_STREAM('\n'
                    << X_opt.rightCols(5) << '\n');
    ROS_INFO("Time: %.5f", (ros::WallTime::now() - begin).toSec());

    nav_msgs::Path ref_path = nav_msgs::Path();
    trajectory_opt::stateToPath(X, ref_path);
    ros::Publisher ref_path_pub;
    ref_path_pub = nh.advertise<nav_msgs::Path>("ref_path", 10);

    nav_msgs::Path opt_path = nav_msgs::Path();
    trajectory_opt::stateToPath(X_opt, opt_path);

    ros::Publisher opt_path_pub;
    opt_path_pub = nh.advertise<nav_msgs::Path>("opt_path", 10);


    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ref_path_pub.publish(ref_path);
        opt_path_pub.publish(opt_path);
        ros::spinOnce();
        loop_rate.sleep();
    }
}