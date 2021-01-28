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


// This section mainly focus on handling the convertion between the ros message and the data type used inside the node

namespace trajectory_opt
{
    //Convert the 2D state into the 3D state representation
    Eigen::Affine3d stateToAffine(const Eigen::Vector3d state);

    //Convert the Eigen Xd Matrix states into the ros message path type 
    void stateToPath(const Eigen::MatrixXd states, nav_msgs::Path &path);

    // Convert the vector of Eigen Xd Matrix into the ros message MultiArray type 
    void VectorEigenMatrixToMsg(const std::vector<Eigen::MatrixXd> & e, std_msgs::Float32MultiArray &m);

    //Convert ros message MultiArray type into the Eigen Xd Matrix
    void msgTomatrixEigen(const std_msgs::Float32MultiArray &msg, Eigen::MatrixXd &e);

    //Conver the Eigen Xd Matrix into the ros message MultiArray type
    void matrixEigenToMsg(const Eigen::MatrixXd &e, std_msgs::Float32MultiArray &m);
} // namespace trajectory_opt
#endif