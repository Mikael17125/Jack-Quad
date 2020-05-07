#ifndef _KINEMATIC_H
#define _KINEMATIC_H

#include <eigen3/Eigen/Eigen>
class Kinematic
{
public:
    Kinematic();
    ~Kinematic();
    double COXA_LEG;
    double TIBIA_LEG;
    double BODY_LENGTH;
    double BODY_WIDTH;
};

#endif