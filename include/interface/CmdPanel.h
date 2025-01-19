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
    float a=0;
    float b=0;
    float x=0;
    float y=0;
    int xx=0;
    int yy=0;
    UserValue(){
        setZero();
    }
    void setZero(){
        lx = 0;
        ly = 0;
        rx = 0;
        ry = 0;
        a = 0;
        b = 0;
        x = 0;
        y = 0;
        xx = 0;
        yy = 0;
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
        /**
         * @file WirelessHandle.cpp
         * @brief 该文件包含了无线处理接口的实现细节。
         * 
         * 本文件主要负责处理与无线设备相关的操作，包括但不限于连接、断开、数据传输等。
         * 具体的函数实现细节在文件内部定义。
         */
        virtual void* run(void *arg){return NULL;}
};

#endif  // CMDPANEL_H