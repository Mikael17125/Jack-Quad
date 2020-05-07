#include "bezier_walk/bezierwalk.h"

BezierWalk::BezierWalk()
{
    module_name_ = "bezier_walk";
}

BezierWalk::~BezierWalk()
{
    queue_thread_.join();
    ROS_INFO("DESTRUCTOR");
}

void BezierWalk::queueThread()
{
    ros::NodeHandle nh_;
    ros::CallbackQueue callback_queue;

    nh_.setCallbackQueue(&callback_queue);
    ROS_INFO("INITIALIZE");
    fr_coxa = nh_.advertise<std_msgs::Float32>("/jack/fr_coxa/command", 1);
    ros::WallDuration duration(8 / 1000.0);

    while (nh_.ok())
        callback_queue.callAvailable(duration);
}

void BezierWalk::initialize()
{
    ROS_INFO("INITIALIZE");
    queue_thread_ = boost::thread(boost::bind(&BezierWalk::queueThread, this));
}

void BezierWalk::process()
{
    ROS_INFO("BEZIER_WALK");
    servoPublish();
}

void BezierWalk::servoPublish()
{
    std_msgs::Float32 init_msg;
    init_msg.data = 1;

    fr_coxa.publish(init_msg);
    // fl_coxa.publish(init_msg);
    // br_coxa.publish(init_msg);
    // bl_coxa.publish(init_msg);
    // fr_tibia.publish(init_msg);
    // fl_tibia.publish(init_msg);
    // br_tibia.publish(init_msg);
    // bl_tibia.publish(init_msg);
}

void BezierWalk::frCoxaCallback(const std_msgs::Float32::ConstPtr &msg)
{
    ROS_INFO("CALLBACK");
}