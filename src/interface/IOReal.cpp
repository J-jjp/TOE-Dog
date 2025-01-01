/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "interface/IOReal.h"
// #include "interface/KeyBoard.h"
// #include "interface/WirelessHandle.h"
#include <iostream>
#include <unistd.h>
#include <csignal>



IOReal::~IOReal(){
    delete cmdPanel;
}

void IOReal::sendRecv( LowlevelCmd *cmd, LowlevelState *state) {
    recv(state);
    std::cout<<"pose"<<cmd->motorCmd[0].Kd;
    send(cmd);
    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
}
void IOReal::send( LowlevelCmd *lowCmd) {
    for (size_t i = 0; i < 12; i++)
    {
        std::cout<<"\t第"<<i<<"条"<<lowCmd->motorCmd[i].q;
    }
    
    std::cout << "send" << std::endl;
    for (size_t i = 0; i < 4; i++)
    {
        std::vector<float> leg_kp(3), leg_kd(3),leg_q(3), leg_dq(3), leg_tau(3);
        for (size_t n = 0; n < 3; n++)
        {
            leg_kp[n] = lowCmd->motorCmd[i*3+n].Kp;
            leg_kd[n] = lowCmd->motorCmd[i*3+n].Kd;
            leg_q[n] = lowCmd->motorCmd[i*3+n].q;
            leg_dq[n]= lowCmd->motorCmd[i*3+n].dq;
            leg_tau[n] = lowCmd->motorCmd[i*3+n].tau;
        }
        legs[i]->set_motor_cmd(leg_kp, leg_kd, leg_q, leg_dq, leg_tau);
    }
}
void IOReal::recv(LowlevelState *state){
    for (int i = 0; i < 4; i++)
    {
        for (size_t n = 0; n < 3; n++)
        {
            state->motorState[i*3+n].q = FL_leg->leg_pose[n];
            state->motorState[i*3+n].dq = FL_leg->leg_W[n];
            state->motorState[i*3+n].tauEst = FL_leg->leg_T[n];
        }
    }
    // for (size_t i = 0; i < 12; i++)
    // {
    //     std::cout<<"\t第"<<i<<"条"<<state->motorState[i].q;
    // }
    // prit_state();
}
void IOReal::prit_state(){
    FL_leg->print_pose();
    // FR_leg->print_pose();
    // RL_leg->print_pose();
    // RR_leg->print_pose();
}