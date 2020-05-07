#ifndef _IK_SOLVER_H
#define _IK_SOLVER_H

#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <Eigen/Dense>
#include <cmath>

#define PI 3.14159265358979323846
#define RAD2DEG 57.272727 //Radian to Degree
#define DEG2RAD 0.01746031746 //Degree to Radian 

struct Kinematic
{
    double coxa_leg = 0.125;
    double tibia_leg = 0.125;
};

class IKSolver
{
public:
    Eigen::Vector2d solve(Eigen::Vector3d coordinate);

private:
    bool checkDomain(double value);
    Eigen::Vector2d last_joint;
    Kinematic k;
};

#endif