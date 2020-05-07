#include "bezier_walk/IKSolver.h"

bool IKSolver::checkDomain(double value)
{
    if (std::fabs(value) <= 1)
        return 1;
    else
        return 0;
}

Eigen::Vector2d IKSolver::solve(Eigen::Vector3d coordinate)
{
    double Z = std::sqrt(std::pow(coordinate.y(), 2) + std::pow(coordinate.z(), 2));

    double A1 = (std::pow(k.coxa_leg, 2) + std::pow(k.tibia_leg, 2) - std::pow(Z, 2)) / (2 * k.coxa_leg * k.tibia_leg);
    double A2 = (std::pow(Z, 2) + std::pow(k.coxa_leg, 2) - std::pow(k.tibia_leg, 2)) / (2 * k.coxa_leg * Z);

    if (!checkDomain(A1) && !checkDomain(A2)) //check domain cos, because domain must have to -1 untill 1
    {
        std::cout << "OUT OF DOMAIN" << std::endl;
        return last_joint;
    }

    double joint_tibia = std::acos(A1);
    double psi = std::acos(A2);
    double alpha = std::atan2(coordinate.z(), coordinate.y());
    double joint_coxa = PI - (alpha + psi);

    last_joint.x() = joint_coxa;
    last_joint.y() = joint_tibia;

    return last_joint;
}