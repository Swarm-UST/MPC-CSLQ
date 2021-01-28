#ifndef ILQR_PLANNER_SERVICE_H
#define ILQR_PLANNER_SERVICE_H
#include <ros/ros.h>
#include <trajectory_opt/OptimizeTrajectory.h>
#include "trajectory_opt/ilqr.h"
#include "trajectory_opt/ros_conversion.h"
#include "std_msgs/Float32MultiArray.h"
#include "eigen_conversions/eigen_msg.h"
#include <string>
#include <Eigen/Dense>
#include <vector>
#include "trajectory_opt/constraint.h"
#include "trajectory_opt/ilqr.h"
namespace trajectory_opt
{
    class IlqrService
    {
    private:
        ros::ServiceServer optimize_trajectory_service_;

    public:
        IlqrService(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        bool optimize_trajectory(OptimizeTrajectory::Request &request, OptimizeTrajectory::Response &response);
        void error(OptimizeTrajectory::Response &response, std::string msg);
    };

} // namespace trajectory_opt

#endif
