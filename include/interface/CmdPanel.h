/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef CMDPANEL_H
#define CMDPANEL_H

#include "common/enumClass.h"
#include <pthread.h>

struct UserValue{
    float lx;
    float ly;
    float rx;
    float ry;
    UserValue(){
        setZero();
    }
    void setZero(){
        lx = 0;
        ly = 0;
        rx = 0;
        ry = 0;
    }
};

class CmdPanel{
    public:
        CmdPanel(){}
        virtual ~CmdPanel(){}
        UserCommand getUserCmd(){return userCmd;}
        UserValue getUserValue(){return userValue;}
        void setPassive(){userCmd = UserCommand::PASS;}
        void setZero(){userValue.setZero();}
    protected:
        virtual void* run(void *arg){return NULL;}
        UserCommand userCmd;
        UserValue userValue;
};

#endif  // CMDPANEL_H