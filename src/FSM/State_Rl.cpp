/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Rl.h"
State_Rl::State_Rl(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::Rl, " rl model"){

}

void State_Rl::enter(){

}

void State_Rl::run(){
    stateMachine();
    mnnInference();
    for (size_t i = 0; i < 12; i++)
    {
        _lowCmd->motorCmd[i].q = action_cmd[0][i];
    }
    
}

void State_Rl::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
}

FSMStateName State_Rl::checkChange(){
    if(_lowState->userCmd == UserCommand::FIXED){
        return FSMStateName::FIXEDSTAND;
    }
    else if(_lowState->userCmd == UserCommand::PASS){
        return FSMStateName::PASSIVE;
    }
    else{
        return FSMStateName::Rl;
    }
}
void State_Rl::mnnInference()
{
    for(int i=0;i<Num_dof;i++){
        last_lowCmd[i]=_lowCmd->motorCmd[i].q;
    }
    memmove(action_history,action_history+Num_dof,(History_len-1)*Num_dof*sizeof(float));
    memcpy(action_history+(History_len-1)*Num_dof,last_lowCmd,Num_dof*sizeof(float));

    for (size_t i = 0; i < 3; i++)
    {
        obs[0][i] = omega[0] *obs_scales_ang_vel;
        obs[0][i+3] = eu_ang[0] *obs_scales_quat;
    }
    obs[0][6] = 0 * obs_scales_lin_vel;
    obs[0][7] = 0 * obs_scales_lin_vel;
    obs[0][8] = 0 * obs_scales_ang_vel;
    for (size_t i = 0; i < 12; i++)
    {
        obs[0][9+i] = (_lowState->motorState[i].q-default_dof_pos[i]) *obs_scales_dof_pos;
        obs[0][21+i] = _lowState->motorState[i].dq * obs_scales_dof_vel;
        obs[0][33+i] = last_lowCmd[i];
    }
    rlptr->advanceNNsync(obs,action_cmd);
}

void State_Rl::stateMachine(){
    if (rlptr == nullptr)
    {
      std::string mobModelPath = "../go2.mnn";
      rlptr = std::make_shared<rl_Inference>(mobModelPath, 50, false);
      rlptr->initBuffer();
    }
    rlptr->resetNode();
}