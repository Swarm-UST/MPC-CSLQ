#ifndef ROS_CONVERSION_H
#define ROS_CONVERSION_H


#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
namespace trajectory_opt
{
    Eigen::Affine3d stateToAffine(const Eigen::Vector3d state);
    void stateToPath(const Eigen::MatrixXd states, nav_msgs::Path &path);
}
#endif