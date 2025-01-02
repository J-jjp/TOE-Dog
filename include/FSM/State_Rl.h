/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef RL_H
#define RL_H

#include "FSM/FSMState.h"
#include <../MNN/Interpreter.hpp>
#include <../MNN/Tensor.hpp>
using namespace MNN;
class State_Rl : public FSMState{
public:
    State_Rl(CtrlComponents *ctrlComp);
    ~State_Rl(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
    void initBuffer();
protected:
    float* currentActionPtr = nullptr;
    float* lastActionPtr = nullptr;
    std::shared_ptr<Interpreter> _net = nullptr;
    Session* _session = nullptr;
    // Tensor* obs_mnn = nullptr;
    // Tensor* act_mnn = nullptr;
    int obs_dim;
    int llc_step = 0;

};

#endif  // FREESTAND_H