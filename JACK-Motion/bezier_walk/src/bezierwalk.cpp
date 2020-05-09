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
    // ROS_INFO("BEZIER_WALK");
    loadConfig();
    motionDemo();
}

void BezierWalk::loadConfig()
{
    std::string config_file = ros::package::getPath("bezier_walk") + "/config/demo.yaml";
    YAML::Node config;

    try
    {
        config = YAML::LoadFile(config_file);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Fail to load yaml file.");
        return;
    }

    YAML::Node node = config["demo"];

    demo.left = node["left"].as<double>();
    demo.right = node["right"].as<double>();
    demo.front = node["front"].as<double>();
    demo.back = node["back"].as<double>();

    demo.speed = node["speed"].as<double>();
}

void BezierWalk::motionDemo()
{
    static int phase = 0;
    static bool inc = false;
    double time_now;
    Eigen::VectorXd joint_goal(8);
    static double time_start = ros::Time::now().toSec();

    time_now = ros::Time::now().toSec() - time_start;

    switch (phase)
    {
    case 0: //Wait untill 5s
        pos.x() = 0.0;
        pos.y() = 0.0;
        pos.z() = 0.005;

        ori.x() = 0.0;
        ori.y() = 0.0;
        ori.z() = 0.0;

        if (time_now > 2)
            phase = 1;

        break;

    case 1: //Stand
        if (inc)
            pos.z() += 0.001;

        if (pos.z() >= 0.11)
            phase = 2;

        break;

    case 2: //Front
        if (inc)
            ori.x() += 0.5 * DEG2RAD;

        if (ori.x() >= demo.front * DEG2RAD)
            phase = 3;

        break;

    case 3: //Back
        if (inc)
            ori.x() -= 0.5 * DEG2RAD;

        if (ori.x() <= demo.back * DEG2RAD)
            phase = 4;

        break;

    case 4: //Normal
        if (inc)
            ori.x() += 0.5 * DEG2RAD;

        if (ori.x() >= 0 * DEG2RAD)
            phase = 5;

        break;

    case 5: //Left
        if (inc)
            ori.y() += 0.5 * DEG2RAD;

        if (ori.y() >= demo.left * DEG2RAD)
            phase = 6;

        break;

    case 6: //Right
        if (inc)
            ori.y() -= 0.5 * DEG2RAD;

        if (ori.y() <= demo.right * DEG2RAD)
            phase = 7;

        break;

    case 7: //Normal
        if (inc)
            ori.y() += 0.5 * DEG2RAD;

        if (ori.y() >= 0 * DEG2RAD)
            phase = 1;

        break;
    }

    joint_goal = ik.solve(pos, ori);

    if (time_now > demo.speed && phase != 0)
    {
        inc = true;
        time_start = ros::Time::now().toSec();
    }
    else
    {
        inc = false;
    }

    for (int i = 0; i < 8; i++)
    {
        // ROS_INFO("JOINT GOAL [%f]", joint_goal(i) * RAD2DEG);
        joint_msg[i].data = joint_goal(i);
        joint_pub[i].publish(joint_msg[i]);
    }
}
