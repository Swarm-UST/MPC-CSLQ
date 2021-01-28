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
    // multiarray(i,j,k) refers to the ith row, jth column, and kth channel.
    void matrixEigenToMsg(const Eigen::MatrixXd &e, std_msgs::Float32MultiArray &m)
    {
        if (m.layout.dim.size() != 2)
            m.layout.dim.resize(2);
    
        m.layout.dim[0].stride = e.rows() * e.cols();
        m.layout.dim[0].size = e.rows();

        m.layout.dim[1].stride = e.cols();
        m.layout.dim[1].size = e.cols();

        if ((int)m.data.size() != e.size())
            m.data.resize(e.size());
        int ii = 0;
        for (int i = 0; i < m.layout.dim[0].size; i++)
            for (int j = 0; j < m.layout.dim[1].size; j++)
                m.data[ii++] = e(i,j);

    }

    void VectorEigenMatrixToMsg(const std::vector<Eigen::MatrixXd> & e, std_msgs::Float32MultiArray &m)
    {

        if (m.layout.dim.size() != 3)
            m.layout.dim.resize(3);
        
        int rows_ = e[0].rows();
        int columns_ = e[0].cols();
        int length_ = e.size();

        m.layout.dim[0].stride = length_*rows_*columns_;
        m.layout.dim[0].size = rows_;

        m.layout.dim[1].stride = length_*columns_;
        m.layout.dim[1].size = columns_;

        m.layout.dim[2].stride = length_;
        m.layout.dim[2].size = length_;     

        if ((int)m.data.size() != m.layout.dim[0].stride)
            m.data.resize(m.layout.dim[0].stride);

        int ii = 0;
        for (int i = 0; i < m.layout.dim[0].size;i++)
            for (int j = 0; j < m.layout.dim[1].size; j++)
                for (int k = 0; k < m.layout.dim[2].size; k++) 
                    m.data[ii++] = e[k](i,j);
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