#include "trajectory_opt/ilrqr_planner_service.h"
#include "std_msgs/Float64MultiArray.h"
#include "eigen_conversions/eigen_msg.h"

namespace trajectory_opt
{
    IlqrService::IlqrService(ros::NodeHandle& nh, ros::NodeHandle& pnh, IlqrPlanner &planner):planner_(planner)
    {
        optimize_trajectory_service_ = nh.advertiseService("optimize_trajectory",&IlqrService::optimize_trajectory, this);
    }
    IlqrService::optimize_trajectory(OptimizeTrajectory::Request &request, OptimizeTrajectory::Response &response)
    {
        
    }
}