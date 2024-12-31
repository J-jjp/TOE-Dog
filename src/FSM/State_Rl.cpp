/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Rl.h"
State_Rl::State_Rl(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::FREESTAND, "free stand"){

}

void State_Rl::enter(){

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
