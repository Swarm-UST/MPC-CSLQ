#ifndef P_CONTROLLER_H
#define P_CONTROLLER_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>


Eigen::Vector2d p_controller(Eigen::Vector3d x,Eigen::Vector3d tar, double l,double r);
#endif