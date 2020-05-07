#include <ros/package.h>
#include <ros/callback_queue.h>

#include "jack_controller/jack_controller.h"

JackController::JackController()
    : is_timer_running_(false)
{
}

bool JackController::initialize()
{
    queue_thread_ = boost::thread(boost::bind(&JackController::msgQueueThread, this));
    return true;
}

void JackController::msgQueueThread()
{
    ros::NodeHandle ros_node;
    ros::CallbackQueue callback_queue;

    ros_node.setCallbackQueue(&callback_queue);
    ROS_INFO("UDA ASWW");

    test_pub = ros_node.advertise<std_msgs::String>("/jack/test", 1);

    ros::Subscriber gazebo_joint_states_sub = ros_node.subscribe("jack/sub", 10,
                                                                 &JackController::gazeboJointStatesCallback, this);

    ros::WallDuration duration(8 / 1000.0);
    while (ros_node.ok())
        callback_queue.callAvailable(duration);
}

void JackController::startTimer()
{
    if (this->is_timer_running_ == true)
        return;

    gazebo_thread_ = boost::thread(boost::bind(&JackController::gazeboTimerThread, this));

    this->is_timer_running_ = true;
}

void JackController::gazeboTimerThread()
{
    ros::Rate gazebo_rate(1000 / 8);

    while (true)
    {
        process();
        gazebo_rate.sleep();
    }
}

void JackController::process()
{
    static bool is_process_running = false;
    if (is_process_running == true)
        return;
    is_process_running = true;

    ROS_INFO("JACK CONTROLLER");


    for(auto module_it = motion_modules_.begin(); module_it != motion_modules_.end(); module_it++){
        (*module_it)->process();
    }

    std_msgs::String msg;
    msg.data = "GA";

    test_pub.publish(msg);
    ROS_INFO(test_pub.getTopic().c_str());

    is_process_running = false;
}

void JackController::addMotionModule(MotionModule *module)
{
    for (auto m_it = motion_modules_.begin(); m_it != motion_modules_.end(); m_it++)
    {
        if ((*m_it)->getModuleName() == module->getModuleName())
        {
            ROS_ERROR("Motion Module Name [%s] already exist !!", module->getModuleName().c_str());
            return;
        }
    }

    module->initialize();
    motion_modules_.push_back(module);
    motion_modules_.unique();
}

void JackController::gazeboJointStatesCallback(const std_msgs::String::ConstPtr &msg)
{
    queue_mutex_.lock();

    ROS_INFO("CALLBACK [%s]",msg->data.c_str());

    queue_mutex_.unlock();
}