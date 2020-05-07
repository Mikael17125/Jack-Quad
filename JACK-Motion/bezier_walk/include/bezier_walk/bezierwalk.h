#ifndef BEZIERWALK_H
#define BEZIERWALK_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <jack_framework_common/motion_module.h>
#include "std_msgs/Float32.h"
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <chrono>

class BezierWalk : public MotionModule, public Singleton<BezierWalk>
{
public:
    BezierWalk();
    virtual ~BezierWalk();
    void process();
    void initialize();

private:
    void queueThread();
    void servoPublish();
    boost::thread queue_thread_;
    ros::Publisher fr_coxa;
    ros::Subscriber fr_coxa_sub;
    void frCoxaCallback(const std_msgs::Float32::ConstPtr &msg);
    // ros::Publisher fl_coxa;
    // ros::Publisher br_coxa;
    // ros::Publisher bl_coxa;
    // ros::Publisher fr_tibia;
    // ros::Publisher fl_tibia;
    // ros::Publisher br_tibia;
    // ros::Publisher bl_tibia;
};

#endif