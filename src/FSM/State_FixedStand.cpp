/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include "FSM/State_FixedStand.h"

State_FixedStand::State_FixedStand(CtrlComponents *ctrlComp)
                :FSMState(ctrlComp, FSMStateName::FIXEDSTAND, "fixed stand"){}

void State_FixedStand::enter(){
    for(int i=0; i<4; i++){
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::Mujoco){
            
            _lowCmd->setSimStanceGain(i);
            _duration=100;
        }
        else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
            _lowCmd->setRealStanceGain(i);
            _duration=400;
        }
    }
    if (_lowCmd->motorCmd[0].q==0&&_lowCmd->motorCmd[1].q==0&&_lowCmd->motorCmd[2].q==0)
    {
        for(int i=0; i<12; i++){
            _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
            _startPos[i] = _lowState->motorState[i].q;
        }
    }
    else{
        for(int i=0; i<12; i++){
            _defpos[i]=_lowCmd->motorCmd[i].q-_lowState->motorState[i].q;
            _lowCmd->motorCmd[i].q = _lowState->motorState[i].q+_defpos[i];
            _startPos[i] = _lowState->motorState[i].q+_defpos[i];
        }
    }
    // for(int i=0; i<12; i++){
    //     _defpos[i]=_lowCmd->motorCmd[i].q-_lowState->motorState[i].q;
    //     _lowCmd->motorCmd[i].q = _lowState->motorState[i].q+ _defpos[i];
    //     _startPos[i] = _lowState->motorState[i].q- _defpos[i];
    // }
    std::cout<<"enter fixed stand";
    // std::cout<<"startPos: "<<;
    _ctrlComp->setAllStance();
}

void State_FixedStand::run(){
    _percent += (float)1/_duration;
    _percent = _percent > 1 ? 1 : _percent;
    for(int j=0; j<12; j++){
        _lowCmd->motorCmd[j].q = (1 - _percent)*_startPos[j] + _percent*_targetPos[j]; 
    }
}

void State_FixedStand::exit(){
    _percent = 0;
    for (size_t i = 0; i < 12; i++)
    {

        _defpos[i]=0;
        _startPos[i]=0;
    }
}

FSMStateName State_FixedStand::checkChange(){
    // if(_lowState->userCmd == UserCommand::L2_B)
    if(_lowState->userCmd == UserCommand::PASS){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::FREE){
        return FSMStateName::FREESTAND;
    }
    else if(_lowState->userCmd == UserCommand::RL){
        return FSMStateName::Rl;
    }
    // else if(_lowState->userCmd == UserCommand::L1_X){
    //     return FSMStateName::BALANCETEST;
    // }
    // else if(_lowState->userCmd == UserCommand::L1_A){
    //     return FSMStateName::SWINGTEST;
    // }
    else{
        return FSMStateName::FIXEDSTAND;
    }
}
