#include "bezier_walk/bezierwalk.h"

BezierWalk::BezierWalk()
{
    module_name_ = "bezier_walk";
}

BezierWalk::~BezierWalk()
{
    queue_thread_.join();
}

void BezierWalk::queueThread()
{
    ros::NodeHandle nh_;
    ros::CallbackQueue callback_queue;

    nh_.setCallbackQueue(&callback_queue);

    for (int i = 0; i < 8; i++)
        joint_pub[i] = nh_.advertise<std_msgs::Float64>("/jack/" + joint_name[i] + "/command", 1);

    ros::WallDuration duration(8 / 1000.0);

    while (nh_.ok())
        callback_queue.callAvailable(duration);
}

void BezierWalk::initialize()
{
    ROS_INFO("INITIALIZE");
    queue_thread_ = boost::thread(boost::bind(&BezierWalk::queueThread, this));

    joint_name[0] = "fl_coxa";
    joint_name[1] = "fr_coxa";
    joint_name[2] = "bl_coxa";
    joint_name[3] = "br_coxa";

    joint_name[4] = "fl_tibia";
    joint_name[5] = "fr_tibia";
    joint_name[6] = "bl_tibia";
    joint_name[7] = "br_tibia";
}

void BezierWalk::process()
{
    ROS_INFO("BEZIER_WALK");
    servoPublish();
}

void BezierWalk::servoPublish()
{
    Eigen::Vector3d pos;
    Eigen::Vector3d ori;
    Eigen::VectorXd joint_goal(8);
    std_msgs::Float64 joint_msg[8];

    // double inc;
    // static double time_start = ros::Time::now().toSec();

    ori.x() = PI/18;
    ori.y() = PI/18;
    ori.z() = 0.0;

    pos.x() = 0.0;
    pos.y() = 0.0;
    pos.z() = 0.1;

    joint_goal = ik.solve(pos, ori);

    // double time_now = ros::Time::now().toSec() - time_start;

    // if (time_now > 0.002)
    // {
    //     inc += DEG2RAD;
    //     time_start = ros::Time::now().toSec();
    // }

    // if (inc <= 1)
    // {
    //     coxa_msg.data = (1 - inc) * 0 + (inc) * (coxa_goal);
    //     tibia_msg.data = (1 - inc) * 0 + (inc) * (tibia_goal);
    // }
    // else
    // {
    //      coxa_msg.data = coxa_goal;
    //      tibia_msg.data = tibia_goal;
    // }

    for (int i = 0; i < 8; i++){
        ROS_INFO("JOINT GOAL [%f]", joint_goal(i) * RAD2DEG);
        joint_msg[i].data = joint_goal(i);
        joint_pub[i].publish(joint_msg[i]);
    }
    


    // ROS_INFO("JOINT COXA [%f]", coxa_msg.data);
    // ROS_INFO("JOINT TIBIA [%f]", tibia_msg.data);
}
