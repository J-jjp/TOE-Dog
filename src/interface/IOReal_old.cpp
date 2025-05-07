/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "interface/IOReal_old.h"
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
    // for (size_t i = 0; i < 12; i++)
    // {
    //     std::cout<<"\t第"<<i<<"条"<<lowCmd->motorCmd[i].q;
    // }
    std::vector<float> FL_leg_kp(3), FL_leg_kd(3),FL_leg_q(3),FL_leg_dq(3), FL_leg_tau(3);
    for (size_t n = 0; n < 3; n++)
    {
        FL_leg_kp[n] = lowCmd->motorCmd[n].Kp;
        FL_leg_kd[n] = lowCmd->motorCmd[n].Kd;
        FL_leg_q[n] = lowCmd->motorCmd[n].q;
        FL_leg_dq[n]= lowCmd->motorCmd[n].dq;
        FL_leg_tau[n] = lowCmd->motorCmd[n].tau;
    }
    FL_leg->set_motor_cmd(FL_leg_kp,FL_leg_kd,FL_leg_q,FL_leg_dq,FL_leg_tau);
    std::vector<float> FR_leg_kp(3), FR_leg_kd(3),FR_leg_q(3),FR_leg_dq(3), FR_leg_tau(3);
    for (size_t n = 0; n < 3; n++)
    {
        FR_leg_kp[n] = lowCmd->motorCmd[n+3].Kp;
        FR_leg_kd[n] = lowCmd->motorCmd[n+3].Kd;
        FR_leg_q[n] = lowCmd->motorCmd[n+3].q;
        FR_leg_dq[n]= lowCmd->motorCmd[n+3].dq;
        FR_leg_tau[n] = lowCmd->motorCmd[n+3].tau;
    }
    FR_leg->set_motor_cmd(FR_leg_kp,FR_leg_kd,FR_leg_q,FR_leg_dq,FR_leg_tau);
    // FR_leg->print_pose();
    std::vector<float> RL_leg_kp(3), RL_leg_kd(3),RL_leg_q(3),RL_leg_dq(3), RL_leg_tau(3);
    for (size_t n = 0; n < 3; n++)
    {
        RL_leg_kp[n] = lowCmd->motorCmd[n+6].Kp;
        RL_leg_kd[n] = lowCmd->motorCmd[n+6].Kd;
        RL_leg_q[n] = lowCmd->motorCmd[n+6].q;
        RL_leg_dq[n]= lowCmd->motorCmd[n+6].dq;
        RL_leg_tau[n] = lowCmd->motorCmd[n+6].tau;
    }
    RL_leg->set_motor_cmd(RL_leg_kp,RL_leg_kd,RL_leg_q,RL_leg_dq,RL_leg_tau);
    std::vector<float> RR_leg_kp(3), RR_leg_kd(3),RR_leg_q(3),RR_leg_dq(3), RR_leg_tau(3);
    for (size_t n = 0; n < 3; n++)
    {
        RR_leg_kp[n] = lowCmd->motorCmd[n+9].Kp;
        RR_leg_kd[n] = lowCmd->motorCmd[n+9].Kd;
        RR_leg_q[n] = lowCmd->motorCmd[n+9].q;
        RR_leg_dq[n]= lowCmd->motorCmd[n+9].dq;
        RR_leg_tau[n] = lowCmd->motorCmd[n+9].tau;
    }
    RR_leg->set_motor_cmd(RR_leg_kp,RR_leg_kd,RR_leg_q,RR_leg_dq,RR_leg_tau);
    // std::cout << "send" << std::endl;

}
void IOReal::recv(LowlevelState *state){
    leg_rec(state);

    for (size_t i = 2; i < 3; i++)
    {
        std::cout<<"\t\t第"<<i<<state->motorState[i].dq;
    }
    std::cout<<std::endl;
    // std::cout<<"motor"<<FL<<"出现"<<leg_pose[moto->_n]<<"\t"<<moto->max_pose
    //     <<"\t"<<leg_pose[moto->_n]<<"\t"<<moto->min_pose<<std::endl;
    // prit_state();
}
void IOReal::prit_state(){
    FL_leg->print_pose();
    // FR_leg->print_pose();
    // RL_leg->print_pose();
    // RR_leg->print_pose();
}
void IOReal::leg_rec(LowlevelState *state){
    for (size_t n = 0; n < 3; n++)
    {
        state->motorState[n].q = FL_leg->leg_pose[n];
        state->motorState[n].dq = FL_leg->leg_W[n]/9.1;
        state->motorState[n].tauEst = FL_leg->leg_T[n]*9.1;
    }
    // state->motorState[1].dq = -state->motorState[1].dq ;
    // state->motorState[1].tauEst = -state->motorState[1].tauEst ;



    state->motorState[3].q = FR_leg->leg_pose[0];
    state->motorState[3].dq = FR_leg->leg_W[0]/9.1;
    state->motorState[3].tauEst = FR_leg->leg_T[0]*9.1;
    for (size_t n = 1; n < 3; n++)
    {
        state->motorState[n+3].q = FR_leg->leg_pose[n];
        state->motorState[n+3].dq = -FR_leg->leg_W[n]/9.1;
        state->motorState[n+3].tauEst = -FR_leg->leg_T[n]*9.1;
    }
    state->motorState[6].q = RL_leg->leg_pose[0];
    state->motorState[6].dq = -RL_leg->leg_W[0]/9.1;
    state->motorState[6].tauEst = -RL_leg->leg_T[0]*9.1;
    for (size_t n = 1; n < 3; n++)
    {
        state->motorState[n+6].q = RL_leg->leg_pose[n];
        state->motorState[n+6].dq = RL_leg->leg_W[n]/9.1;
        state->motorState[n+6].tauEst = RL_leg->leg_T[n]*9.1;
    } 
    for (size_t n = 0; n < 3; n++)
    {
        state->motorState[n+9].q = RR_leg->leg_pose[n];
        state->motorState[n+9].dq = -RR_leg->leg_W[n]/9.1;
        state->motorState[n+9].tauEst = -RR_leg->leg_T[n]*9.1;
    }
}
void IOReal::thread_send(std::shared_ptr<leg_control>  leg,std::vector<float>& leg_kp,std::vector<float>& leg_kd,
std::vector<float>& leg_q,std::vector<float>& leg_dq,std::vector<float>& leg_tau){
    leg->set_motor_cmd(leg_kp,leg_kd,leg_q,leg_dq,leg_tau);
}