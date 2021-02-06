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
            double corner_radius = 0.1;
            Eigen::Vector2d center_to_cur_vec = cur_pos - center_pos;
            double angle = atan2(center_to_cur_vec(1), center_to_cur_vec(0));
            double cut_off_angle = atan2(height, width);
            double top_cut_off_angle = atan2(height,width-corner_radius);
            double side_cut_off_angle = atan2(height-corner_radius,width);
            if (angle >= -M_PI + cut_off_angle && angle < -cut_off_angle) //bottom
            {
                if (angle >= - top_cut_off_angle) // Bottom right corner
                {
                   Eigen::Vector2d from_center_vec = cur_pos - Eigen::Vector2d(left_bottom_pos(0)+width-corner_radius,left_bottom_pos(1)+corner_radius);
                   return (from_center_vec)*(from_center_vec.norm()-corner_radius)/from_center_vec.norm();
                }
                else if (angle <= -M_PI + top_cut_off_angle) // Bottom left corner
                {
                   Eigen::Vector2d from_center_vec = cur_pos - Eigen::Vector2d(left_bottom_pos(0)+corner_radius,left_bottom_pos(1)+corner_radius);
                   return (from_center_vec)*(from_center_vec.norm()-corner_radius)/from_center_vec.norm();
                }
                else
                {
                    return Eigen::Vector2d(0, cur_pos(1) - (left_bottom_pos(1)));
                }
            }
            else if (angle >= -cut_off_angle && angle < cut_off_angle) //right
            {
                if (angle <= - side_cut_off_angle) // Bottom right corner
                {
                   Eigen::Vector2d from_center_vec = cur_pos - Eigen::Vector2d(left_bottom_pos(0)+width-corner_radius,left_bottom_pos(1)+corner_radius);
                   return (from_center_vec)*(from_center_vec.norm()-corner_radius)/from_center_vec.norm();
                }
                else if (angle >= side_cut_off_angle) // Top right corner
                {
                   Eigen::Vector2d from_center_vec = cur_pos - Eigen::Vector2d(left_bottom_pos(0)+width-corner_radius,left_bottom_pos(1)+height-corner_radius);
                   return (from_center_vec)*(from_center_vec.norm()-corner_radius)/from_center_vec.norm();
                }
                else
                {
                    return Eigen::Vector2d(cur_pos(0) - (left_bottom_pos(0) + width), 0);
                }
                
            }
            else if (angle >= cut_off_angle && angle < M_PI - cut_off_angle) //top
            {
                if (angle >= M_PI - top_cut_off_angle) // Top left corner
                {
                   Eigen::Vector2d from_center_vec = cur_pos - Eigen::Vector2d(left_bottom_pos(0)+corner_radius,left_bottom_pos(1)+height-corner_radius);
                   return (from_center_vec)*(from_center_vec.norm()-corner_radius)/from_center_vec.norm();
                }
                else if (angle <= top_cut_off_angle) // Top right corner
                {
                   Eigen::Vector2d from_center_vec = cur_pos - Eigen::Vector2d(left_bottom_pos(0)+width-corner_radius,left_bottom_pos(1)+height-corner_radius);
                   return (from_center_vec)*(from_center_vec.norm()-corner_radius)/from_center_vec.norm();
                }
                else
                {
                    return  Eigen::Vector2d(0, cur_pos(1) - (left_bottom_pos(1) + height));
                }
                
            }
            else //left
            {
                if (angle <= M_PI - side_cut_off_angle && angle > 0) // Top left corner
                {
                   Eigen::Vector2d from_center_vec = cur_pos - Eigen::Vector2d(left_bottom_pos(0)+corner_radius,left_bottom_pos(1)+height-corner_radius);
                   return (from_center_vec)*(from_center_vec.norm()-corner_radius)/from_center_vec.norm();
                }
                else if (angle >= - M_PI + side_cut_off_angle && angle < 0) // Bottom left corner
                {
                   Eigen::Vector2d from_center_vec = cur_pos - Eigen::Vector2d(left_bottom_pos(0)+corner_radius,left_bottom_pos(1)+corner_radius);
                   return (from_center_vec)*(from_center_vec.norm()-corner_radius)/from_center_vec.norm();
                }
                else
                {
                    return Eigen::Vector2d(cur_pos(0) - (left_bottom_pos(0)), 0);
                }
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
            double corner_radius = 0.1;

            // check whether it is inside four circle at the corner
            // Top right corner
            if((cur_pos - Eigen::Vector2d(left_bottom_pos(0)+width-corner_radius,left_bottom_pos(1)+height-corner_radius)).norm() <= corner_radius)
            {
                return true;
            }// Top left corner
            else if ((cur_pos - Eigen::Vector2d(left_bottom_pos(0)+corner_radius,left_bottom_pos(1)+height-corner_radius)).norm() <= corner_radius)
            {
                return true;
            }// Bottom right corner
            else if ((cur_pos - Eigen::Vector2d(left_bottom_pos(0)+width-corner_radius,left_bottom_pos(1)+corner_radius)).norm() <= corner_radius)
            {       
                return true;
            }// Bottom left corner
            else if ((cur_pos - Eigen::Vector2d(left_bottom_pos(0)+corner_radius,left_bottom_pos(1)+corner_radius)).norm() <= corner_radius)
            {
                return true;
            }// wide rectangle 
            else if (x >= x_ref && x <= x_ref + width && y >= y_ref + corner_radius && y <= y_ref + height -corner_radius)
            {
                return true;
            }// tall rectangle
            else if (x >= x_ref + corner_radius && x <= x_ref + width - corner_radius && y >= y_ref  && y <= y_ref + height )
            {
                return true;
            }
            else
            {
               return false;
            }
                
            
        }
    }
} // namespace trajectory_opt
