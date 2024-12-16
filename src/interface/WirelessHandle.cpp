/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef USE_WIRELESS_HANDLE

#include "interface/WirelessHandle.h"
#include "common/mathTools.h"
#include <string.h>
#include <stdio.h>

WirelessHandle::WirelessHandle(ros::NodeHandle &n){
    _joy_sub = n.subscribe("/joy", 10, &WirelessHandle::joyCallback, this);
}

void WirelessHandle::joyCallback(const sensor_msgs::Joy::ConstPtr &joy){
    if(joy->buttons[5] && joy->buttons[1]) {
        userCmd = UserCommand::L2_B;
    }
    else if(joy->buttons[5] && joy->buttons[0]) {
        userCmd = UserCommand::L2_A;
    }
    else if(joy->buttons[5] && joy->buttons[2]) {
        userCmd = UserCommand::L2_X;
    }



    else if(joy->buttons[4] && joy->buttons[2]) {
        userCmd = UserCommand::L1_X;
    }
    else if(joy->buttons[4] && joy->buttons[0]) {
        userCmd = UserCommand::L1_A;
    }
    else if(joy->buttons[4] && joy->buttons[3]) {
        userCmd = UserCommand::L1_Y;
    }
    else if(joy->buttons[7]) {
        userCmd = UserCommand::START;
    }

    userValue.lx = killZeroOffset(-joy->axes[0], 0.08);
    userValue.ly = killZeroOffset(joy->axes[1], 0.08);
    userValue.rx = killZeroOffset(-joy->axes[3], 0.08);
    userValue.ry = killZeroOffset(joy->axes[4], 0.08);
}

#endif  // USE_WIRELESS_HANDLE
