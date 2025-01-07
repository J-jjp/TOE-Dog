/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Rl.h"
State_Rl::State_Rl(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::Rl, " rl model"){

}

void State_Rl::enter(){
    for(int i=0; i<4; i++){
        _lowCmd->setSimrlGain(i);
        _lowCmd->setZeroTau(i);
        // else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
        //     _lowCmd->setRealrlGain(i);
        // }
    }
    for (size_t i = 0; i < Num_observations; i++)
    {
        policy_input[0][i]=0;
    }
    for (size_t i = 0; i < Num_observations; i++)
    {
        last_lowCmd[i]=0;
    }
    for (size_t i = 0; i < History_len*N_proprio; i++)
    {
        obs_history[0][i]=0;
    }
}

void State_Rl::run(){
    stateMachine();
    mnnInference();
    
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
    // for(int i=0;i<Num_dof;i++){
    //     last_lowCmd[i]=_lowCmd->motorCmd[i].q;
    // }
    // memmove(action_history,action_history+Num_dof,(History_len-1)*Num_dof*sizeof(float));
    // memcpy(action_history+(History_len-1)*Num_dof,last_lowCmd,Num_dof*sizeof(float));
    Vec3 eu_ang;
    eu_ang = quaternion_to_euler_array(_lowState->imu.getQuat());
    for (size_t i = 0; i < 3; i++)
    {
        obs[0][i] = _lowState->imu.gyroscope[i] *obs_scales_ang_vel;
        obs[0][i+3] = eu_ang[0] *obs_scales_quat;
    }
    obs[0][6] = -_lowState->userValue.ly * obs_scales_lin_vel;
    float y_vel=0;
    if (_lowState->userValue.lx >0.9)
    {
        y_vel = _lowState->userValue.ly+0.5;
    }
    if (_lowState->userValue.lx <-0.9)
    {
        y_vel = _lowState->userValue.ly-0.5;
    }
    
    obs[0][7] = -y_vel * obs_scales_lin_vel;
    obs[0][8] = _lowState->userValue.rx * obs_scales_ang_vel;
    for (size_t i = 0; i < 12; i++)
    {
        obs[0][9+i] = (_lowState->motorState[i].q-default_dof_pos[i]) *obs_scales_dof_pos;
        obs[0][21+i] = _lowState->motorState[i].dq * obs_scales_dof_vel;
        obs[0][33+i] = last_lowCmd[i];
    }
    for (size_t i = 0; i < N_proprio; i++)
    {
        policy_input[0][i] = obs[0][i];
    }
    for (size_t i = 0; i < N_priv_latent  + N_scan; i++)
    {
        policy_input[0][i+N_proprio]=0;
    }
    for (size_t i = 0; i < History_len*N_proprio; i++)
    {
        policy_input[0][i+N_proprio+N_priv_latent+N_scan]=obs_history[0][i];
    }
    for (size_t i = 0; i < (History_len-1)*N_proprio; i++)
    {
        obs_history[0][i] = obs_history[0][i+N_proprio];
    }
    for (size_t i = 0; i < N_proprio; i++)
    {
        obs_history[0][((History_len-1)*N_proprio)+i] = obs[0][i];
    }
    rlptr->advanceNNsync(policy_input,action_cmd);
    for (size_t i = 0; i < 12; i++)
    {
        std::cout<<"action_cmd"<<action_cmd[0][i];
    }
    std::cout<<std::endl;
    float action_flt[12];
    for (size_t i = 0; i < 12; i++)
    {
        action_flt[i]=action_cmd[0][i]*0.8+last_lowCmd[i]*0.2;
    }
    for (size_t i = 0; i < 12; i++)
    {
        last_lowCmd[i]=action_cmd[0][i];
    }
    for (size_t i = 0; i < 12; i++)
    {
        _lowCmd->motorCmd[i].q = action_flt[i] * 0.25 + default_dof_pos[i];
    }
    // _lowCmd->motorCmd[6].q = 0.0;
    // _lowCmd->motorCmd[7].q = 3.0;
    // _lowCmd->motorCmd[8].q = 3.0;
}

void State_Rl::stateMachine(){
    if (rlptr == nullptr)
    {
      std::string mobModelPath = "../go2.mnn";
      rlptr = std::make_shared<rl_Inference>(mobModelPath);
      rlptr->initBuffer();
    }
    rlptr->resetNode();
}
Vec3 State_Rl::quaternion_to_euler_array(Vec4 quat){
    double x = quat[1];
    double y = quat[2];
    double z = quat[3];
    double w = quat[0];
    double t0, t1, t2, t3,t4;
    double roll_x, pitch_y, yaw_z;
    t0 = +2.0 * (w * x + y * z);
    t1 = +1.0 - 2.0 * (x * x + y * y);
    roll_x = std::atan2(t0, t1);
    
    t2 = +2.0 * (w * y - z * x);
    t2 = std::max(-1.0, std::min(t2, 1.0));
    pitch_y =  std::asin(t2);
    
    t3 = +2.0 * (w * z + x * y);
    t4 = +1.0 - 2.0 * (y * y + z * z);
    yaw_z =  std::atan2(t3, t4);
    return {roll_x, pitch_y, yaw_z};
}