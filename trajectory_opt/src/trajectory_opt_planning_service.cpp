#include <ros/ros.h>
#include "trajectory_opt/ilrqr_planner_service.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_service");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ROS_INFO("Node start\n");
    trajectory_opt::IlqrService ilrq_server(nh, pnh);

    ros::spin();
}