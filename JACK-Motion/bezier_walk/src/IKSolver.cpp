#include "bezier_walk/IKSolver.h"

IKSolver::IKSolver()
{
    FL << k.BODY_WIDTH / 2, k.BODY_LENGTH / 2, 0, 0;
    FR << -k.BODY_WIDTH / 2, k.BODY_LENGTH / 2, 0, 0;
    BL << k.BODY_WIDTH / 2, -k.BODY_LENGTH / 3, 0, 0;
    BR << -k.BODY_WIDTH / 2, -k.BODY_LENGTH / 3, 0, 0;

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

Eigen::VectorXd IKSolver::solve(Eigen::Vector3d pos, Eigen::Vector3d ori)
{
    Eigen::Vector4d pos_;
    Eigen::Vector3d fl_pos, fr_pos, bl_pos, br_pos;
    Eigen::Vector2d ik_fl, ik_fr, ik_bl, ik_br;
    Eigen::VectorXd result(8);

    pos_ << pos.x(), pos.y(), pos.z(), 0;

    //CoM to Feet
    pos_FL = pos_ + FL;
    pos_FR = pos_ + FR;
    pos_BL = pos_ + BL;
    pos_BR = pos_ + BR;

    //Transform Body
    mFL = transform(pos, ori) * FL;
    mFR = transform(pos, ori) * FR;
    mBL = transform(pos, ori) * BL;
    mBR = transform(pos, ori) * BR;

    //CoM to Feet (New)
    mFL = pos_FL - mFL;
    mFR = pos_FR - mFR;
    mBL = pos_BL - mBL;
    mBR = pos_BR - mBR;

    //Untransform
    mFL = transform(-pos, -ori) * mFL;
    mFR = transform(-pos, -ori) * mFR;
    mBL = transform(-pos, -ori) * mBL;
    mBR = transform(-pos, -ori) * mBR;

    fl_pos << mFL(0), mFL(1), mFL(2);
    fr_pos << mFR(0), mFR(1), mFR(2);
    bl_pos << mBL(0), mBL(1), mBL(2);
    br_pos << mBR(0), mBR(1), mBR(2);

    ik_fl = analyticIK(fl_pos);
    ik_fr = analyticIK(fr_pos);
    ik_bl = analyticIK(bl_pos);
    ik_br = analyticIK(br_pos);
    
    result << ik_fl(0), ik_fr(0), ik_bl(0), ik_br(0),
              ik_fl(1), ik_fr(1), ik_bl(1), ik_br(1);

    return result;
}