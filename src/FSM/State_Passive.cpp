/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "State_Passive.h"

State_Passive::State_Passive(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::PASSIVE, "passive"){}

void State_Passive::enter(){
    if(_ctrlComp->ctrlPlatform == CtrlPlatform::Mujoco){
        std::cout<<"faijf";
        for(int i=0; i<12; i++){
            _lowCmd->motorCmd[i].q = 0;
            _lowCmd->motorCmd[i].dq = 0;
            _lowCmd->motorCmd[i].Kp = 0;    
            _lowCmd->motorCmd[i].Kd = 0.1;
            _lowCmd->motorCmd[i].tau = 0;
        }
    }
    else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
#ifdef ROBOT_TYPE_T1
        for(int i=0; i<4; i++){
            _lowCmd->motorCmd[3*i].q = 0;
            _lowCmd->motorCmd[3*i].dq = 0;
            _lowCmd->motorCmd[3*i].Kp = 0;
            _lowCmd->motorCmd[3*i].Kd = 6;
            _lowCmd->motorCmd[3*i].tau = 0;
        }
        for(int i=0; i<4; i++){
            _lowCmd->motorCmd[3*i+1].q = 0;
            _lowCmd->motorCmd[3*i+1].dq = 0;
            _lowCmd->motorCmd[3*i+1].Kp = 0;
            _lowCmd->motorCmd[3*i+1].Kd = 6;
            _lowCmd->motorCmd[3*i+1].tau = 0;
        }
        for(int i=0; i<4; i++){
            _lowCmd->motorCmd[3*i+2].q = 0;
            _lowCmd->motorCmd[3*i+2].dq = 0;
            _lowCmd->motorCmd[3*i+2].Kp = 0;
            _lowCmd->motorCmd[3*i+2].Kd = 6;
            _lowCmd->motorCmd[3*i+2].tau = 0;
        }
#endif
#ifdef ROBOT_TYPE_T2
        for(int i=0; i<4; i++){
            _lowCmd->motorCmd[3*i].q = 0;
            _lowCmd->motorCmd[3*i].dq = 0;
            _lowCmd->motorCmd[3*i].Kp = 0;
            _lowCmd->motorCmd[3*i].Kd = 30;
            _lowCmd->motorCmd[3*i].tau = 0;
        }
        for(int i=0; i<4; i++){
            _lowCmd->motorCmd[3*i+1].q = 0;
            _lowCmd->motorCmd[3*i+1].dq = 0;
            _lowCmd->motorCmd[3*i+1].Kp = 0;
            _lowCmd->motorCmd[3*i+1].Kd = 35;
            _lowCmd->motorCmd[3*i+1].tau = 0;
        }
        for(int i=0; i<4; i++){
            _lowCmd->motorCmd[3*i+2].q = 0;
            _lowCmd->motorCmd[3*i+2].dq = 0;
            _lowCmd->motorCmd[3*i+2].Kp = 0;
            _lowCmd->motorCmd[3*i+2].Kd = 40;
            _lowCmd->motorCmd[3*i+2].tau = 0;
        }
#endif
    }
    _ctrlComp->setAllSwing();
}

void State_Passive::run(){
    
}

void State_Passive::exit(){

}

FSMStateName State_Passive::checkChange(){
    if(_lowState->userCmd == UserCommand::FIXED){
        std::cout<<"change";
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::PASSIVE;
    }
}