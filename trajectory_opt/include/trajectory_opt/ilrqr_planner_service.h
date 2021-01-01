#ifndef ILQR_PLANNER_SERVICE_H
#define ILQR_PLANNER_SERVICE_H
#include <ros/ros.h>
#include <trajectory_opt/OptimizeTrajectory.h>
#include "trajectory_opt/ilqr_planner.h"

namespace trajectory_opt
{
    class IlqrService
    {
    private:
        ros::SericeServer optimize_trajectory_service_;
        IlqrPlanner planner_;

    public:
        IlqrService(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        optimize_trajectory(OptimizeTrajectory::Request &request, OptimizeTrajectory::Response &response);
    }
} // namespace trajectory_opt

#endif