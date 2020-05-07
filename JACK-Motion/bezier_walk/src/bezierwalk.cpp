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
    std_msgs::Float64 coxa_msg;
    std_msgs::Float64 tibia_msg;

    static double coxa_now = 0.0;
    static double tibia_now = 0.0;
    static double time_start = ros::Time::now().toSec();

    double coxa_goal = 30.0;
    double tibia_goal = 60.0;

    double time_now = ros::Time::now().toSec() - time_start;
    
    if (time_now > 0.01)
    {
        coxa_now += 1;
        tibia_now += 2;
        time_start = ros::Time::now().toSec();
    }

    ROS_INFO("TIME NOW [%f]", time_now);

    if (coxa_now <= coxa_goal)
        coxa_msg.data = coxa_now * DEG2RAD;
    else
        coxa_msg.data = coxa_goal * DEG2RAD;
    
    if (tibia_now <= tibia_goal)
        tibia_msg.data = tibia_now * DEG2RAD;
    else
        tibia_msg.data = tibia_goal * DEG2RAD;
        
    for (int i = 0; i < 8; i++)
    {
        if (i < 4)
            joint_pub[i].publish(coxa_msg);
        else
            joint_pub[i].publish(tibia_msg);
    }
}
