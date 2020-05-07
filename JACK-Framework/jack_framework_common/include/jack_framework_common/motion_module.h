#ifndef JACK_FRAMEWORK_COMMON_MOTION_MODULE_H_
#define JACK_FRAMEWORK_COMMON_MOTION_MODULE_H_

#include <map>
#include <string>

#include "singleton.h"

class MotionModule
{
protected:
    std::string module_name_;

public:
    virtual ~MotionModule() {}
    std::string getModuleName() { return module_name_; }
    
    virtual void initialize() = 0;
    virtual void process() = 0;
};

#endif