/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef State_RL_H
#define State_RL_H

#include "FSM/FSMState.h"
#include "control/rl_Inference.h"
#define Num_dof 12
#define N_scan 187
#define N_priv_latent  4 + 1 + 12 + 12 + 12 + 6 + 1 + 4 + 1 - 3 + 4
#define N_proprio  45  
#define History_len 10
#define Num_observations  (N_proprio + N_priv_latent + N_scan + (History_len * N_proprio))
class State_Rl : public FSMState{
public:
    State_Rl(CtrlComponents *ctrlComp);
    ~State_Rl(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
    void State_Rl::mnnInference();
    std::shared_ptr<rl_Inference> rlptr = nullptr;
    float last_lowCmd[Num_dof] = {0};
    float action_history[History_len*Num_dof]; //使用六帧前的数据
    float obs_history[Num_observations];
};

#endif  // FREESTAND_H