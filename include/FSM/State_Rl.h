/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef RL_H
#define RL_H

#include "FSM/FSMState.h"

class State_Rl : public FSMState{
public:
    State_Rl(CtrlComponents *ctrlComp);
    ~State_Rl(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
private:

};

#endif  // FREESTAND_H