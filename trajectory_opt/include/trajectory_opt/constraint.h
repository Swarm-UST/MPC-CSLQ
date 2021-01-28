#ifndef CONSTRAINT_H_
#define CONSTRAINT_H_

#include <Eigen/Dense>
#include <cmath>

namespace trajectory_opt
{
    enum constraint_type
    {
        RECTANGLE = 1,
        CIRCLE
    };

    class Constraint
    {
    public:
        //Left bottoom
        Constraint(Eigen::Vector2d left_bottom_pos, double width, double height);
        Constraint(Eigen::Vector2d center_pos, double radius);

        Eigen::Vector2d getDisplacementFromBoundary(const Eigen::Vector2d cur_pos) const;
        bool isInConstraint(const Eigen::Vector2d cur_pos) const;

        const constraint_type type;
        Eigen::Vector2d left_bottom_pos;
        double width, height;
        Eigen::Vector2d center_pos;
        double radius;
    };
} // namespace trajectory_opt
#endif