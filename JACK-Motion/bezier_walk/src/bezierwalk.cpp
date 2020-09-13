#include "bezier_walk/bezierwalk.h"

BezierWalk::BezierWalk()
    : fb_active(false)
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

    ros::Subscriber imu_callback = nh_.subscribe("/jack/imu", 10, &BezierWalk::ImuCallback, this);

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
    feedbackPID();
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

    demo.foot_height = node["foot_height"].as<double>();
    demo.foot_step = node["foot_step"].as<double>();
    demo.freq = node["freq"].as<double>();

    demo.feedback = node["feedback"].as<bool>();
    demo.speed = node["speed"].as<double>();

    demo.KP_R = node["KP_R"].as<double>();
    demo.KP_P = node["KP_P"].as<double>();

    demo.KD_R = node["KD_R"].as<double>();
    demo.KD_P = node["KD_P"].as<double>();
}

void BezierWalk::feedbackPID()
{
    Eigen::Vector3d feedback;

    if (demo.feedback)
    {
        if (std::fabs(imu.x()) > 1 * DEG2RAD && std::fabs(imu.x()) < 40 * DEG2RAD && fb_active)
        {
            feedback.x() = demo.KP_R * imu.x() + demo.KD_R * d_imu.x();
        }
        else
        {
            feedback.x() = 0;
        }

        if (std::fabs(imu.y()) > 1 * DEG2RAD && std::fabs(imu.y()) < 40 * DEG2RAD && fb_active)
        {
            feedback.y() = -1 * (demo.KP_P * imu.y() + demo.KD_P * d_imu.y());
        }
        else
        {
            feedback.y() = 0;
        }

        feedback.z() = 0;
    }
    else
    {
        feedback.x() = 0;
        feedback.y() = 0;
        feedback.z() = 0;
    }

    // ROS_INFO("FEEDBACK ACTIVE X [%f]", feedback.x());
    // ROS_INFO("FEEDBACK ACTIVE Y [%f]", feedback.y());

    pos.x() = demo.pos_x;
    pos.y() = demo.pos_y;
    pos.z() = demo.pos_z;

    ori.x() = demo.ori_x * DEG2RAD + feedback.x();
    ori.y() = demo.ori_y * DEG2RAD + feedback.y();
    ori.z() = demo.ori_z * DEG2RAD + feedback.z();
}

void BezierWalk::motionDemo()
{
    static double phase_one = 0;
    static double phase_two = 0;
    static bool delay;
    static int support = 0;
    static bool inc = false;
    double time_now;
    Eigen::VectorXd joint_goal(8);
    Eigen::MatrixXd goal_pos(4, 3);
    static double time_start = ros::Time::now().toSec();

    time_now = ros::Time::now().toSec() - time_start;

    if (phase_one > 1)
        time_start = ros::Time::now().toSec();

    phase_one = time_now / demo.freq;

    if (phase_one < 0.5)
        phase_two = ((time_now + (demo.freq / 2)) / demo.freq);
    else
        phase_two = ((time_now - (demo.freq / 2)) / demo.freq);

    // left = bezierCurve3(phase_one, Eigen::Vector3d(0.0, 0.0, 0.1), Eigen::Vector3d(demo.foot_step, 0.0, 0.1 + demo.foot_height), Eigen::Vector3d(0.0, 0.0, 0.1));
    // right = bezierCurve3(phase_two, Eigen::Vector3d(0.0, 0.0, 0.1), Eigen::Vector3d(demo.foot_step, 0.0, 0.1 + demo.foot_height), Eigen::Vector3d(0.0, 0.0, 0.1));

    switch (support)
    {
    case 0:
        // ROS_ERROR("AAAAAAAAAAAAAAAAAAAAAA");
        left = bezierCurve3(phase_one, Eigen::Vector3d(0.0, 0.0, 0.1), Eigen::Vector3d(demo.foot_step / 2, 0.0, 0.1 + demo.foot_height), Eigen::Vector3d(demo.foot_step, 0.0, 0.1));
        right = bezierCurve3(phase_one, Eigen::Vector3d(demo.foot_step, 0.0, 0.1), Eigen::Vector3d(demo.foot_step / 2, 0.0, 0.1), Eigen::Vector3d(0.0, 0.0, 0.1));

        if (phase_one < 1)
            delay = false;

        if (phase_one > 1 && !delay)
        {
            delay = true;
            support = 1;
        }

        break;

    case 1:
        // ROS_ERROR("BBBBBBBBBBBBBBBBBBBBBBB");
        left = bezierCurve3(phase_one, Eigen::Vector3d(demo.foot_step, 0.0, 0.1), Eigen::Vector3d(demo.foot_step / 2, 0.0, 0.1), Eigen::Vector3d(0.0, 0.0, 0.1));
        right = bezierCurve3(phase_one, Eigen::Vector3d(0.0, 0.0, 0.1), Eigen::Vector3d(demo.foot_step / 2, 0.0, 0.1 + demo.foot_height), Eigen::Vector3d(demo.foot_step, 0.0, 0.1));

        if (phase_one < 1)
            delay = false;

        if (phase_one > 1 && !delay)
        {
            delay = true;
            support = 0;
        }

        break;
    }

    goal_pos << 0.0625 + left.x(), 0.1 + left.x(), left.z(),
        -0.0625 + right.x(), 0.1 + right.x(), right.z(),
        0.0625 + right.x(), -0.15 + right.x(), right.z(),
        -0.0625 + left.x(), -0.15 + left.x(), left.z();

    joint_goal = ik.solve(pos, ori, goal_pos);

    for (int i = 0; i < 8; i++)
    {
        // ROS_INFO("JOINT GOAL [%s] = [%f]", joint_name[i].c_str(), joint_goal(i) * RAD2DEG);
        joint_msg[i].data = joint_goal(i);
        joint_pub[i].publish(joint_msg[i]);
    }
}

void BezierWalk::ImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{

    Eigen::Quaterniond imu_quarternion;

    imu_quarternion.x() = msg->orientation.x;
    imu_quarternion.y() = msg->orientation.y;
    imu_quarternion.z() = msg->orientation.z;
    imu_quarternion.w() = msg->orientation.w;

    Eigen::Matrix3d rotation;

    rotation = imu_quarternion.toRotationMatrix();

    imu.x() = atan2(rotation.coeff(2, 1), rotation.coeff(2, 2));
    imu.y() = atan2(-rotation.coeff(2, 0), sqrt(pow(rotation.coeff(2, 1), 2) + pow(rotation.coeff(2, 2), 2)));
    imu.z() = atan2(rotation.coeff(1, 0), rotation.coeff(0, 0));

    d_imu.x() = msg->angular_velocity.x;
    d_imu.y() = msg->angular_velocity.y;
    d_imu.z() = msg->angular_velocity.z;
}

Eigen::Vector3d BezierWalk::bezierCurve2(double phase, Eigen::Vector3d start, Eigen::Vector3d end)
{
    Eigen::Vector3d result;

    result(0) = (1 - phase) * start(0) + phase * end(0);
    result(1) = (1 - phase) * start(1) + phase * end(1);
    result(2) = (1 - phase) * start(2) + phase * end(2);

    return result;
}

Eigen::Vector3d BezierWalk::bezierCurve3(double phase, Eigen::Vector3d start, Eigen::Vector3d inter, Eigen::Vector3d end)
{
    Eigen::Vector3d result;

    result(0) = std::pow((1 - phase), 2) * start(0) + 2 * (1 - phase) * phase * inter(0) + std::pow(phase, 2) * end(0);
    result(1) = std::pow((1 - phase), 2) * start(1) + 2 * (1 - phase) * phase * inter(1) + std::pow(phase, 2) * end(1);
    result(2) = std::pow((1 - phase), 2) * start(2) + 2 * (1 - phase) * phase * inter(2) + std::pow(phase, 2) * end(2);

    return result;
}
