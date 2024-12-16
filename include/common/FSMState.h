/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include <unistd.h>
#include "control/CtrlComponents.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "common/enumClass.h"
#include "common/mathTools.h"
#include "common/mathTypes.h"
#include "common/timeMarker.h"
#include "interface/CmdPanel.h"

class FSMState{
public:
    FSMState(CtrlComponents *ctrlComp, FSMStateName stateName, std::string stateNameString);

    virtual void enter() = 0;//纯虚函数 子类必须重新实现
    virtual void run() = 0;
    virtual void exit() = 0;
    virtual FSMStateName checkChange() {return FSMStateName::INVALID;} //虚函数  子类可以重新实现

    FSMStateName _stateName; //用以区分是哪个状态
    std::string _stateNameString;//状态的名称 保存于一个字符串
protected:
    CtrlComponents *_ctrlComp;//该类包括了大部分的控制
    FSMStateName _nextStateName;//保存下一个状态

    LowlevelCmd *_lowCmd;//电机发送所用变量
    LowlevelState *_lowState;//电机状态所用变量
    UserValue _userValue;  //一些用户值,包括遥控值、按键值
};

#endif  // FSMSTATE_H