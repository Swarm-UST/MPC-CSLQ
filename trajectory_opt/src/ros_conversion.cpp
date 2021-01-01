#include "trajectory_opt/ros_conversion.h"

namespace trajectory_opt
{
    Eigen::Affine3d stateToAffine(const Eigen::Vector3d state)
    {
        Eigen::Affine3d t;
        t = Eigen::Translation3d(state(0),state(1),0);
        t = t.rotate(Eigen::AngleAxisd(state(2), Eigen::Vector3d(0,0,1)));
        return t;
    }
    void stateToPath(const Eigen::MatrixXd states, nav_msgs::Path &path)
    {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "world";
        path_msg.header.stamp = ros::Time::now();
        for(int i=0;i < states.cols();i++)
        {
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header = path_msg.header;
            Eigen::Affine3d s = stateToAffine(states.col(i));
            tf::poseEigenToMsg(s, pose_msg.pose);
            path_msg.poses.push_back(pose_msg);
        }
        path = path_msg;
    }
}