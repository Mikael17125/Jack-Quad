#ifndef BEZIERWALK_H
#define BEZIERWALK_H

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <ros/callback_queue.h>
#include <jack_framework_common/motion_module.h>
#include <bezier_walk/IKSolver.h>
#include <bezier_walk/trajectory.h>
#include <bezier_walk/kinematic.h>
#include "yaml-cpp/yaml.h"

#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

#include <eigen3/Eigen/Eigen>
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

#define RAD2DEG 57.272727     //Radian to Degree
#define DEG2RAD 0.01746031746 //Degree to Radian

using namespace Eigen;

struct Demo
{
    double ori_x;
    double ori_y;
    double ori_z;

    double pos_x;
    double pos_y;
    double pos_z;

    double foot_height;
    
    double speed;
};

class BezierWalk : public MotionModule,
                   public Singleton<BezierWalk>
{
public:
    BezierWalk();
    virtual ~BezierWalk();
    void process();
    void initialize();

private:
    IKSolver ik;
    Trajectory traj;
    Kinematic k;

    void queueThread();
    void loadConfig();
    void motionDemo();

    void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg);

    boost::thread queue_thread_;
    ros::Publisher joint_pub[8];
    std::string joint_name[8];

    Eigen::Vector3d pos;
    Eigen::Vector3d ori;

    double roll, pitch, yaw;

    //Message
    std_msgs::Float64 joint_msg[8];

    Demo demo;
};

#endif