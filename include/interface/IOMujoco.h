/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef IOMUJOCO_H
#define IOMUJOCO_H

#include "interface/IOInterface.h"
#include <mujoco/mujoco.h>
#include <string>
class IOMujoco : public IOInterface{
public:
    IOMujoco(mjData *data):_data(data){
        std::cout<<"generate interfaces"<<std::endl;
    }
    ~IOMujoco(){};
    void sendRecv(LowlevelCmd *cmd, LowlevelState *state);
    void sendRecv_debug(LowlevelCmd *cmd, LowlevelState *state,float kp,float kd);
    void send(LowlevelCmd *cmd);
    void recv(LowlevelState *state);
private:
    mjData* _data;
    float pd_control(float target_q,float q,float kp,float target_dq,float dq,float kd);
};

#endif