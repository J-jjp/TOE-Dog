/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/CmdPanel.h"
#include <string>
#include <vector>
class IOInterface{
public:
    IOInterface(){}
    ~IOInterface(){
        delete cmdPanel;
        }
    virtual void sendRecv(LowlevelCmd *cmd, LowlevelState *state){};
    //liu tao add---------------------------------
    virtual void send(LowlevelCmd *cmd){};
    virtual void recv(LowlevelState *state){};
    virtual void sendRecv_debug(LowlevelCmd *cmd, LowlevelState *state,float kp,float kd){};
    //liu tao add---------------------------------
    void zeroCmdPanel(){cmdPanel->setZero();}
    void setPassive(){cmdPanel->setPassive();}

    CmdPanel *cmdPanel;
};

#endif  //IOINTERFACE_H