#include "trajectory_opt/constraint.h"

namespace trajectory_opt
{
    Constraint::Constraint(Eigen::Vector2d left_bottom_pos, double width, double height) : type(constraint_type::RECTANGLE), left_bottom_pos(left_bottom_pos), width(width), height(height)
    {
        Eigen::Vector2d displace_vec(width, height);
        Eigen::Vector2d right_top_pos = left_bottom_pos + displace_vec;
        center_pos = (left_bottom_pos + right_top_pos) / 2.0;
    }
    Constraint::Constraint(Eigen::Vector2d center_pos, double radius) : type(constraint_type::CIRCLE), center_pos(center_pos), radius(radius)
    {
    }
    Eigen::Vector2d Constraint::getDisplacementFromBoundary(const Eigen::Vector2d cur_pos) const
    {
        if (type == constraint_type::CIRCLE)
        {
            Eigen::Vector2d from_center_vec = cur_pos - center_pos;
            return (from_center_vec)*(from_center_vec.norm()-radius)/from_center_vec.norm();
        }
        else
        {
            Eigen::Vector2d center_to_cur_vec = cur_pos - center_pos;
            double angle = atan2(center_to_cur_vec(1), center_to_cur_vec(0));
            double cut_off_angle = atan2(height, width);
            if (angle >= -M_PI + cut_off_angle && angle < -cut_off_angle) //bottom
            {
                return Eigen::Vector2d(0, cur_pos(1) - (left_bottom_pos(1)));
            }
            else if (angle >= -cut_off_angle && angle < cut_off_angle) //right
            {
                return Eigen::Vector2d(cur_pos(0) - (left_bottom_pos(0) + width), 0);
            }
            else if (angle >= cut_off_angle && angle < M_PI - cut_off_angle) //top
            {
                return  Eigen::Vector2d(0, cur_pos(1) - (left_bottom_pos(1) + height));
            }
            else //left
            {
                return Eigen::Vector2d(cur_pos(0) - (left_bottom_pos(0)), 0);
            }
        }
    }
    bool Constraint::isInConstraint(const Eigen::Vector2d cur_pos) const
    {
        if (type == constraint_type::CIRCLE)
        {
            return (cur_pos - center_pos).norm() <= radius;
        }
        else
        {
            double x = cur_pos(0), y = cur_pos(1);
            double x_ref = left_bottom_pos(0), y_ref = left_bottom_pos(1);
            if (x >= x_ref && x <= x_ref + width && y >= y_ref && y <= y_ref + height)
                return true;
            else
                return false;
            
        }
    }
} // namespace trajectory_opt
