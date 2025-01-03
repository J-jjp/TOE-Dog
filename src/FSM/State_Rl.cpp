/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Rl.h"
State_Rl::State_Rl(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::Rl, " rl model"){

}

void State_Rl::enter(){
    if (rlptr == nullptr)
    {
      std::string mobModelPath = "../go2.mnn";
      rlptr = std::make_shared<rl_Inference>(mobModelPath, 50, false);
      rlptr->initBuffer();
    }
    rlptr->resetNode();
}

void State_Rl::run(){

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
  float rate = 1;
  float temp_t=0.2;


    for(int i=0;i<NUM_DOFS;i++){
        last_lowCmd[i]=_lowCmd->motorCmd[i].q;
    }

  rlptr->advanceNNsync(obs_history,last_lowCmd);

  //clip action immediately
  float clip_action=10.0;
  for(int i=0;i<12;i++)
  {
    if(action_cmd_[i]<-clip_action)
      action_cmd_[i]=-clip_action;
    if(action_cmd_[i]>clip_action)
      action_cmd_[i]=clip_action;
  }

  // for(int i=0;i<12;i++)
  // {
  //   action_cmd_[i]*=0.25;
  // }
  // action_cmd_[0]*=0.5;
  // action_cmd_[3]*=0.5;
  // action_cmd_[6]*=0.5;
  // action_cmd_[9]*=0.5;

  memmove(action_history,action_history+NUM_DOFS,(ACTION_HISTORY_NUMS-1)*NUM_DOFS*sizeof(float));
  memcpy(action_history+(ACTION_HISTORY_NUMS-1)*NUM_DOFS,last_lowCmd,NUM_DOFS*sizeof(float));

  float action_scaled[12];
  //todo 根据deploy的代码，似乎在发送的时候立刻将当前的action发送出去，并没有实现训练时的模拟滞后操作
  for(int i=0;i<NUM_DOFS;i++)
  {
    // action_scaled[i]=action_history[i];
    action_scaled[i]=action_cmd_[i];
  }

  for(int i=0;i<12;i++)
  {
    action_scaled[i]*=0.25;
  }
  action_scaled[0]*=0.5;
  action_scaled[3]*=0.5;
  action_scaled[6]*=0.5;
  action_scaled[9]*=0.5;

  // update joints order
  for (size_t i = 0; i < 3; i++)
  {
    jCmd[0 + i] = action_scaled[3 + i] + default_joint_state_[3 + i];
    jCmd[3 + i] = action_scaled[0 + i] + default_joint_state_[0 + i];
    jCmd[6 + i] = action_scaled[9 + i] + default_joint_state_[9 + i];
    jCmd[9 + i] = action_scaled[6 + i] + default_joint_state_[6 + i];
  }

  // for(int i=0;i<12;i++)
  //   std::cout<<jCmd[i]<<" ";
  // std::cout<<std::endl;


  for (size_t j = 0; j < NUM_DOFS; ++j)
  {
    cmd.motorCmd[j].q = jCmd[j] * rate + init_joint_state_[j] * (1 - rate);
    cmd.motorCmd[j].dq = 0;
    cmd.motorCmd[j].tau = 0; 
  }
}

