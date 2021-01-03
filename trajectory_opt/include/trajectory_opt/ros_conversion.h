#ifndef ROS_CONVERSION_H
#define ROS_CONVERSION_H

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/ros.h>
namespace trajectory_opt
{
    Eigen::Affine3d stateToAffine(const Eigen::Vector3d state);
    void stateToPath(const Eigen::MatrixXd states, nav_msgs::Path &path);
    void msgTomatrixEigen(const std_msgs::Float32MultiArray &msg, Eigen::MatrixXd &e);
    void matrixEigenToMsg(const Eigen::MatrixXd &e, std_msgs::Float32MultiArray &m);
} // namespace trajectory_opt
#endif