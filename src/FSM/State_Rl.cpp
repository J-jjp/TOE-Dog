/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Rl.h"
State_Rl::State_Rl(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::Rl, " rl model"){
   _net = std::shared_ptr<Interpreter> (Interpreter::createFromFile("../go2.mnn"));
    ScheduleConfig config;
    config.numThread = 2;
    _session = _net->createSession(config);
}

void State_Rl::enter(){
    initBuffer();
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
void State_Rl::initBuffer(){
  obs_mnn = _net->getSessionInput(_session, nullptr);
  act_mnn = _net->getSessionOutput(_session, nullptr);

  obs_dim = obs_mnn->shape().back();
  std::cerr << "obs_dim: " << obs_dim << std::endl;

  currentActionPtr = new float[act_mnn->shape().back()]();
  lastActionPtr = new float[act_mnn->shape().back()]();
  std::cerr << "mnn net init finished" << std::endl;
}
