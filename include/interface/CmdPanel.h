/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef CMDPANEL_H
#define CMDPANEL_H

#include "common/enumClass.h"
#include <pthread.h>

struct UserValue{
    float lx=0;
    float ly=0;
    float rx=0;
    float ry=0;
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
        virtual void read_joy(){}
        UserCommand getUserCmd(){return userCmd;}
        UserValue getUserValue(){return userValue;}
        void setPassive(){userCmd = UserCommand::PASS;}
        void setZero(){userValue.setZero();}
        UserCommand userCmd;
        UserValue userValue;
    protected:
        virtual void* run(void *arg){return NULL;}
};

#endif  // CMDPANEL_H