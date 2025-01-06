/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef IOMUJOCO_H
#define IOMUJOCO_H

#include "interface/IOInterface.h"
#include <mujoco/mujoco.h>
#include <string>
#include "interface/KeyBoard.h"
#include <iomanip>
class IOMujoco : public IOInterface{
public:
    IOMujoco(mjData *data,mjModel* model):_data(data),_model(model){
        std::cout<<"generate interfaces"<<std::endl;
        cmdPanel = new KeyBoard();
    }
    ~IOMujoco(){};
    void sendRecv(LowlevelCmd *cmd, LowlevelState *state);
    // void send(LowlevelCmd *cmd);
    void recv(LowlevelState *state);
private:
    mjData* _data;
    mjModel* _model;
    float pd_control(MotorCmd *cmd,MotorState *state);
    std::vector<mjtNum> get_sensor_data(const std::string &sensor_name);
};

#endif