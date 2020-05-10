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

    demo.pos_x = node["pos_x"].as<double>();
    demo.pos_y = node["pos_y"].as<double>();
    demo.pos_z = node["pos_z"].as<double>();

    demo.ori_x = node["ori_x"].as<double>();
    demo.ori_y = node["ori_y"].as<double>();
    demo.ori_z = node["ori_z"].as<double>();

    demo.speed = node["speed"].as<double>();
}

void BezierWalk::motionDemo()
{
    static int phase = 0;
    static bool inc = false;
    double time_now;
    static double height_1, height_2;
    Eigen::VectorXd joint_goal(8);
    Eigen::MatrixXd goal_pos(4, 3);
    static double time_start = ros::Time::now().toSec();

    time_now = ros::Time::now().toSec() - time_start;

    pos.x() = demo.pos_x;
    pos.y() = demo.pos_y;
    pos.z() = demo.pos_z;

    ori.x() = demo.ori_x * DEG2RAD;
    ori.y() = demo.ori_y * DEG2RAD;
    ori.z() = demo.ori_z * DEG2RAD;

    switch (phase)
    {
    case 0: //Wait untill 2s

        height_1 = 0.005;
        height_2 = 0.005;

        if (time_now > 2)
            phase = 1;

        break;

    case 1: //Stand
        if (inc)
        {
            height_1 += 0.0001;
            height_2 += 0.0001;
        }

        if (height_1 >= 0.11)
            phase = 2;

        break;

    case 2: //FL BR UP
        if (inc)
            height_1 -= 0.01;

        if (height_1 <= 0.04)
            phase = 3;

        break;

    case 3: //FL BR DOWN
        if (inc)
            height_1 += 0.01;

        if (height_1 >= 0.11)
            phase = 4;

        break;

    case 4: //FR BL UP
        if (inc)
            height_2 -= 0.01;

        if (height_2 <= 0.04)
            phase = 5;

        break;

    case 5: //FR BL DOWN
        if (inc)
            height_2 += 0.01;

        if (height_2 >= 0.11)
            phase = 2;

        break;
    }

    goal_pos << 0.0625, 0.15, height_1,
        -0.0625, 0.15, height_2,
        0.0625, -0.1, height_2,
        -0.0625, -0.1, height_1;

    joint_goal = ik.solve(pos, ori, goal_pos);

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
        // ROS_INFO("JOINT GOAL [%s] = [%f]", joint_name[i].c_str(), joint_goal(i) * RAD2DEG);
        joint_msg[i].data = joint_goal(i);
        joint_pub[i].publish(joint_msg[i]);
    }
}
