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
#define obs_scales_lin_vel 2
#define obs_scales_ang_vel 0.25
#define obs_scales_quat 1
#define obs_scales_dof_pos 1
#define obs_scales_dof_vel 0.05
class State_Rl : public FSMState{
public:
    State_Rl(CtrlComponents *ctrlComp);
    ~State_Rl(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
    void mnnInference();
    void stateMachine();
    std::vector<float> default_dof_pos={0.1,0.8,-1.5 ,-0.1,0.8,-1.5,0.1,1,-1.5, -0.1,1.,-1.5};//#默认角度需要与isacc一致
    std::shared_ptr<rl_Inference> rlptr = nullptr;
    float last_lowCmd[Num_dof];
    float action_history[History_len*Num_dof]; //使用六帧前的数据
    float obs_history[1][Num_observations];
    float obs[1][N_proprio];
    float action_cmd[1][Num_dof];
};

#endif  // FREESTAND_H