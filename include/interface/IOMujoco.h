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
    float pd_control(float target_q,float q,float kp,float target_dq,float dq,float kd);
    void sendRecv(LowlevelCmd *cmd, LowlevelState *state,float kp,float kd);
    void send(LowlevelCmd *cmd);
    void recv(LowlevelState *state);
private:
    mjData* _data;
};

#endif  // IOSIM_H