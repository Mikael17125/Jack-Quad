#include "bezier_walk/IKSolver.h"

IKSolver::IKSolver()
{
    FL << k.BODY_WIDTH / 2, k.BODY_LENGTH / 2, 0, 1;
    FR << -k.BODY_WIDTH / 2, k.BODY_LENGTH / 2, 0, 1;
    BL << k.BODY_WIDTH / 2, -k.BODY_LENGTH / 3, 0, 1;
    BR << -k.BODY_WIDTH / 2, -k.BODY_LENGTH / 3, 0, 1;

    Transform.setIdentity();
}

bool IKSolver::checkDomain(double value)
{
    if (std::fabs(value) <= 1)
        return 1;
    else
        return 0;
}

Eigen::Matrix4d IKSolver::transform(Eigen::Vector3d pos, Eigen::Vector3d ori)
{

    R = AngleAxisd(ori.x(), Vector3d::UnitX()) *
        AngleAxisd(ori.y(), Vector3d::UnitY()) *
        AngleAxisd(ori.z(), Vector3d::UnitZ());

    Transform.block<3, 3>(0, 0) = R;
    Transform.block<3, 1>(0, 3) = pos;

    return Transform;
}

Eigen::Vector2d IKSolver::analyticIK(Eigen::Vector3d pos)
{
    double Z = std::sqrt(std::pow(pos.y(), 2) + std::pow(pos.z(), 2));

    double A1 = (std::pow(k.COXA_LEG, 2) + std::pow(k.TIBIA_LEG, 2) - std::pow(Z, 2)) / (2 * k.COXA_LEG * k.TIBIA_LEG);
    double A2 = (std::pow(Z, 2) + std::pow(k.COXA_LEG, 2) - std::pow(k.TIBIA_LEG, 2)) / (2 * k.COXA_LEG * Z);

    if (!checkDomain(A1) && !checkDomain(A2)) //check domain cos, because domain must have to -1 untill 1
    {
        std::cout << "OUT OF DOMAIN" << std::endl;
        return last_joint;
    }

    double joint_tibia = std::acos(A1);
    double psi = std::acos(A2);
    double alpha = std::atan2(pos.z(), pos.y());
    double joint_coxa = PI - (alpha + psi);

    last_joint(0) = joint_coxa;
    last_joint(1) = joint_tibia;

    return last_joint;
}

Eigen::VectorXd IKSolver::solve(Eigen::Vector3d pos, Eigen::Vector3d ori, Eigen::MatrixXd goal)
{
    Eigen::Vector4d goal_fl, goal_fr, goal_bl, goal_br;
    Eigen::Vector3d fl_pos, fr_pos, bl_pos, br_pos;
    Eigen::Vector2d ik_fl, ik_fr, ik_bl, ik_br;
    Eigen::VectorXd result(8);

    //CoM to Feet
    goal_fl << goal(0, 0), goal(0, 1), goal(0, 2), 1;
    goal_fr << goal(1, 0), goal(1, 1), goal(1, 2), 1;
    goal_bl << goal(2, 0), goal(2, 1), goal(2, 2), 1;
    goal_br << goal(3, 0), goal(3, 1), goal(3, 2), 1;

    //Transform Body
    tFL = transform(pos, ori) * FL;
    tFR = transform(pos, ori) * FR;
    tBL = transform(pos, ori) * BL;
    tBR = transform(pos, ori) * BR;

    std::cout << tFL << std::endl<<std::endl;
    std::cout << transform(pos, ori) << std::endl<<std::endl;

    //Coxa to Feet (New)
    goal_fl = goal_fl - tFL;
    goal_fr = goal_fr - tFR;
    goal_bl = goal_bl - tBL;
    goal_br = goal_br - tBR;

    //Untransform
    goal_fl = transform(-pos, -ori) * goal_fl;
    goal_fr = transform(-pos, -ori) * goal_fr;
    goal_bl = transform(-pos, -ori) * goal_bl;
    goal_br = transform(-pos, -ori) * goal_br;

    fl_pos << goal_fl(0), goal_fl(1), goal_fl(2);
    fr_pos << goal_fr(0), goal_fr(1), goal_fr(2);
    bl_pos << goal_bl(0), goal_bl(1), goal_bl(2);
    br_pos << goal_br(0), goal_br(1), goal_br(2);

    ik_fl = analyticIK(fl_pos);
    ik_fr = analyticIK(fr_pos);
    ik_bl = analyticIK(bl_pos);
    ik_br = analyticIK(br_pos);

    result << ik_fl(0), ik_fr(0), ik_bl(0), ik_br(0),
              ik_fl(1), ik_fr(1), ik_bl(1), ik_br(1);

    return result;
}