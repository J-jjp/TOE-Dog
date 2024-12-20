/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FSM_H
#define FSM_H

// FSM States
#include "FSMState.h"
#include "State_FixedStand.h"
#include "State_Passive.h"
#include "State_FreeStand.h"
#include "common/enumClass.h"
#include "control/CtrlComponents.h"
struct FSMStateList{
    FSMState *invalid;
    State_Passive *passive;
    State_FixedStand *fixedStand;
    State_FreeStand *freeStand;
    void deletePtr(){
        delete invalid;
        delete passive;
        delete fixedStand;
        delete freeStand;
    }
};

class FSM{
public:
    FSM(CtrlComponents *ctrlComp);
    ~FSM();
    void initialize();
    void run();//在该函数里执行了整个工程的所有逻辑(如估计器迭代、信息命令收发、状态机运行、状态切换等)
private:
    FSMState* getNextState(FSMStateName stateName);
    bool checkSafty();
    CtrlComponents *_ctrlComp;
    FSMState *_currentState;
    FSMState *_nextState;
    FSMStateName _nextStateName;
    FSMStateList _stateList;
    FSMMode _mode;
    long long _startTime;
    int count;
};


#endif  // FSM_H
