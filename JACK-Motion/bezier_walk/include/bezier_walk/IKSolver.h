#ifndef _IK_SOLVER_H
#define _IK_SOLVER_H

#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

#include <bezier_walk/kinematic.h>

#define PI 3.14159265358979323846
#define RAD2DEG 57.272727     //Radian to Degree
#define DEG2RAD 0.01746031746 //Degree to Radian

using namespace Eigen;

class IKSolver
{
public:
    IKSolver();
    Eigen::VectorXd solve(Eigen::Vector3d pos, Eigen::Vector3d ori);

private:
    Eigen::Matrix4d transform(Eigen::Vector3d pos, Eigen::Vector3d ori);
    Eigen::Vector2d analyticIK(Eigen::Vector3d pos);
    bool checkDomain(double value);
    Kinematic k;

    Eigen::Matrix3d R;
    Eigen::Matrix4d Transform;
    Eigen::Vector4d FL, FR, BL, BR;
    Eigen::Vector4d pos_FL, pos_FR, pos_BL, pos_BR;
    Eigen::Vector4d mFL, mFR, mBL, mBR;
    Eigen::Vector2d last_joint;
    Eigen::Vector2d ik_fl, ik_fr, ik_bl, ik_br;
};

#endif