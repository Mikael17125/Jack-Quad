#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "bezier_walk/bezierwalk.h"

#include <jack_controller/jack_controller.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jack_manager");
    ros::NodeHandle nh_;

    JackController *controller = JackController::getInstance();

    if (controller->initialize() == false)
    {
        ROS_ERROR("JACK Controller Initialize Fail!");
        return -1;
    }
    controller->addMotionModule((MotionModule *)BezierWalk::getInstance());
    usleep(300 * 1000);

    controller->startTimer();

    usleep(100 * 1000);

    while (ros::ok())
    {
        usleep(1 * 1000);

        ros::spin();
    }

    return 0;
}
