#include "trajectory_opt/ros_conversion.h"

namespace trajectory_opt
{
    Eigen::Affine3d stateToAffine(const Eigen::Vector3d state)
    {
        Eigen::Affine3d t;
        t = Eigen::Translation3d(state(0), state(1), 0);
        t = t.rotate(Eigen::AngleAxisd(state(2), Eigen::Vector3d(0, 0, 1)));
        return t;
    }
    void stateToPath(const Eigen::MatrixXd states, nav_msgs::Path &path)
    {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "world";
        path_msg.header.stamp = ros::Time::now();
        for (int i = 0; i < states.cols(); i++)
        {
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header = path_msg.header;
            Eigen::Affine3d s = stateToAffine(states.col(i));
            tf::poseEigenToMsg(s, pose_msg.pose);
            path_msg.poses.push_back(pose_msg);
        }
        path = path_msg;
    }

    void matrixEigenToMsg(const Eigen::MatrixXd &e, std_msgs::Float32MultiArray &m)
    {
        if (m.layout.dim.size() != 2)
            m.layout.dim.resize(2);
        m.layout.dim[0].stride = e.rows() * e.cols();
        m.layout.dim[0].size = e.cols();
        m.layout.dim[1].stride = e.rows();
        m.layout.dim[1].size = e.rows();
        if ((int)m.data.size() != e.size())
            m.data.resize(e.size());
        int ii = 0;
        for (int i = 0; i < e.cols(); ++i)
            for (int j = 0; j < e.rows(); ++j)
                m.data[ii++] = e(j, i);
    }
    void msgTomatrixEigen(const std_msgs::Float32MultiArray &msg, Eigen::MatrixXd &e)
    {
        int columns = msg.layout.dim[0].size;
        int rows = msg.layout.dim[1].size;
        ROS_ERROR_COND((int)msg.data.size() != columns * rows, "msgTomatrixEigen: Wrong dim");
        Eigen::Map<const Eigen::MatrixXf> m_e(msg.data.data(), rows, columns);
        e = m_e.cast<double>();
    }
} // namespace trajectory_opt