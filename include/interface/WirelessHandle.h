/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef WIRELESSHANDLE_H
#define WIRELESSHANDLE_H

#include "interface/CmdPanel.h"
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>

class WirelessHandle : public CmdPanel{
public:
    WirelessHandle(ros::NodeHandle &n);
    ~WirelessHandle(){}
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
private:
    ros::Subscriber _joy_sub;
};

#endif  // WIRELESSHANDLE_H