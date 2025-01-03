/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef IOMUJOCO_H
#define IOMUJOCO_H

#include "interface/IOInterface.h"
#include <mujoco/mujoco.h>
#include <string>
#include "interface/KeyBoard.h"
#include "interface/WirelessHandle.h"
class IOMujoco : public IOInterface{
public:
    IOMujoco(mjData *data):_data(data){
        std::cout<<"generate interfaces"<<std::endl;
        if (0)
        {
            cmdPanel = new WirelessHandle();
        }
        else{
            cmdPanel = new KeyBoard();
        }
    }
    ~IOMujoco();
    void sendRecv(LowlevelCmd *cmd, LowlevelState *state);
    void send(LowlevelCmd *cmd);
    void recv(LowlevelState *state);
private:
    mjData* _data;
    float pd_control(float target_q,float q,float kp,float target_dq,float dq,float kd);
};

#endif