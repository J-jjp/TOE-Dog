/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Rl.h"
State_Rl::State_Rl(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::Rl, " rl model"){

}

void State_Rl::enter(){
    for(int i=0; i<4; i++){
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::Mujoco){
            _lowCmd->setSimrlGain(i);
            _lowCmd->setZeroTau(i);
        }
        else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
            _lowCmd->setRealrlGain(i);
        }
    }
    if(walk_these_ways){
        memset(action_cmd_Walk,0,sizeof(float)*NUM_DOFS);
        memset(last_action_cmd_Walk,0,sizeof(float)*NUM_DOFS);
        memset(action_history,0,sizeof(float)*ACTION_HISTORY_NUMS*NUM_DOFS);
        memset(obs_buf_,0,sizeof(float)*OBS_DIM);
        memset(obs_history_with_adaptation,0,sizeof(float)*(NUM_OBS_IN_OBS_HISTORY*OBS_DIM+2));
        memset(obs_history,0,sizeof(float)*NUM_OBS_IN_OBS_HISTORY*OBS_DIM);

        mobCmd_[0]=0;
        mobCmd_[1]=0;
        mobCmd_[2]=0;
        if(CMD_DIM>=4)mobCmd_[3]=0;
        if(CMD_DIM>=5)mobCmd_[4]=1;
        if(CMD_DIM>=6)mobCmd_[5]=0.5;
        if(CMD_DIM>=7)mobCmd_[6]=0.0;
        if(CMD_DIM>=8)mobCmd_[7]=0.0;
        if(CMD_DIM>=9)mobCmd_[8]=0.5;
        if(CMD_DIM>=10)mobCmd_[9]=0.1;
        if(CMD_DIM>=11)mobCmd_[10]=0.0;
        if(CMD_DIM>=12)mobCmd_[11]=0.0;
        if(CMD_DIM>=13)mobCmd_[12]=0.25;
        if(CMD_DIM>=14)mobCmd_[13]=0.38;
        if(CMD_DIM>=15)mobCmd_[14]=0;


        gait_indices=0.0;
        std::cout << "Mob Loop set up finished" << std::endl;
    }
    else{
        for (size_t i = 0; i < Num_observations_Loco; i++)
        {
            policy_input[0][i]=0;
        }
        for (size_t i = 0; i < Num_observations_Loco; i++)
        {
            last_lowCmd[i]=0;
        }
        for (size_t i = 0; i < History_len_Loco*N_proprio_Loco; i++)
        {
            obs_history[0][i]=0;
        }
    }
}

void State_Rl::run(){
    if (walk_these_ways)
    {
        stateMachine_Walk();
        mnnInference_Walk();
        getCurrentObservation_Walk();
    }
    else{
        stateMachine_Loco();
        mnnInference_Loco();
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
void State_Rl::mnnInference_Loco()
{
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
    for (size_t i = 0; i < N_proprio_Loco; i++)
    {
        policy_input[0][i] = obs[0][i];
    }
    for (size_t i = 0; i < N_priv_latent_Loco  + N_scan_Loco; i++)
    {
        policy_input[0][i+N_proprio_Loco]=0;
    }
    for (size_t i = 0; i < History_len_Loco*N_proprio_Loco; i++)
    {
        policy_input[0][i+N_proprio_Loco+N_priv_latent_Loco+N_scan_Loco]=obs_history[0][i];
    }
    for (size_t i = 0; i < (History_len_Loco-1)*N_proprio_Loco; i++)
    {
        obs_history[0][i] = obs_history[0][i+N_proprio_Loco];
    }
    for (size_t i = 0; i < N_proprio_Loco; i++)
    {
        obs_history[0][((History_len_Loco-1)*N_proprio_Loco)+i] = obs[0][i];
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

void State_Rl::stateMachine_Loco(){
    if (rlptr == nullptr)
    {
      std::string mobModelPath = "../go2.mnn";
      rlptr = std::make_shared<rl_Inference>(mobModelPath);
      rlptr->initBuffer();
    }
    rlptr->resetNode();
}
void State_Rl::mnnInference_Walk()
{
    // for (size_t i = 0; i < NUM_OBS_IN_OBS_HISTORY*OBS_DIM; i++)
    // {
    //    obs_history_Walk[i]=0;
    // }
    
    adaptationNetPtr->advanceNNsync_Walk(obs_history_Walk, adaptation_output);
        //adaptation_output: 7.82271 0.837328
    std::cout<<"adaptation"<<adaptation_output[0]<<"\t"<<adaptation_output[1]<<std::endl;
    memcpy(obs_history_with_adaptation , obs_history_Walk , NUM_OBS_IN_OBS_HISTORY*OBS_DIM * sizeof(float));
    obs_history_with_adaptation[NUM_OBS_IN_OBS_HISTORY*OBS_DIM]=adaptation_output[0];
    obs_history_with_adaptation[NUM_OBS_IN_OBS_HISTORY*OBS_DIM+1]=adaptation_output[1];
    for(int i=0;i<NUM_DOFS;i++)
        last_action_cmd_Walk[i]=action_cmd_Walk[i];
    rlptr->advanceNNsync_Walk(obs_history_with_adaptation, action_cmd_Walk);
    float clip_action=10.0;
    for(int i=0;i<12;i++)
    {
        if(action_cmd_Walk[i]<-clip_action)
        action_cmd_Walk[i]=-clip_action;
        if(action_cmd_Walk[i]>clip_action)
        action_cmd_Walk[i]=clip_action;
    }
    memmove(action_history,action_history+NUM_DOFS,(ACTION_HISTORY_NUMS-1)*NUM_DOFS*sizeof(float));
    memcpy(action_history+(ACTION_HISTORY_NUMS-1)*NUM_DOFS,action_cmd_Walk,NUM_DOFS*sizeof(float));
    float action_scaled[12];
    for(int i=0;i<NUM_DOFS;i++)
    {
        // action_scaled[i]=action_history[i];
        action_scaled[i]=action_cmd_Walk[i];
    }
    for(int i=0;i<12;i++)
    {
        action_scaled[i]*=0.25;
    }
    action_scaled[0]*=0.5;
    action_scaled[3]*=0.5;
    action_scaled[6]*=0.5;
    action_scaled[9]*=0.5;

    for (size_t i = 0; i < 12; i++)
    {
        _lowCmd->motorCmd[i].q = action_scaled[i]+ default_dof_pos[i];
    }
    // for (size_t j = 0; j < NUM_DOFS; ++j)
    // {
    //     cmd.motorCmd[j].q = jCmd[j] * rate + init_joint_state_[j] * (1 - rate);
    //     cmd.motorCmd[j].dq = 0;
    //     cmd.motorCmd[j].tau = 0; 
    // }
}

void State_Rl::stateMachine_Walk(){
    if (rlptr == nullptr)
    {
      std::string mobModelPath = "/home/jjp/dog/TOE-Dog/body_latest.mnn";
      rlptr = std::make_shared<rl_Inference>(mobModelPath);
      rlptr->initBuffer();
    }
    rlptr->resetNode();
    if (adaptationNetPtr == nullptr)
    {
      std::string mobModelPath = "/home/jjp/dog/TOE-Dog/adaptation_module_latest.mnn";
      adaptationNetPtr = std::make_shared<rl_Inference>(mobModelPath);
      adaptationNetPtr->initBuffer();
    }
    mobRun();
}
void State_Rl::mobRun()
{
//   if ((int)joy_msg.buttons[0] == 1 && gait_index != 0)  // trot
//   {
    mobCmd_[5]=0.5;
    mobCmd_[6]=0;
    mobCmd_[7]=0;
    // gait_index = 0;
//   }
//   else if ((int)joy_msg.buttons[1] == 1  && gait_index != 1)  // pace
//   {
//     mobCmd_[5]=0;
//     mobCmd_[6]=0;
//     mobCmd_[7]=0.5;
//     gait_index = 1;
//   }
//   else if ((int)joy_msg.buttons[2] == 1 && gait_index != 2)  // pronk
//   {
//     mobCmd_[5]=0;
//     mobCmd_[6]=0;
//     mobCmd_[7]=0;
//     gait_index = 2;
//   }
//   else if ((int)joy_msg.buttons[3] == 1 && gait_index != 3)  // bound
//   {
//     mobCmd_[5]=0;
//     mobCmd_[6]=0.5;
//     mobCmd_[7]=0;
//     gait_index = 3;
//   }
  //todo stand
  // else if ((int)_keyData.btn.components.left == 1 && gait_index != 5)  // stand
  // {
  //   gait_index = 5;
  // }
//   else if (joy_msg.axes[7]  > 0 )
//   {
//     mobCmd_[4] += 0.5;
//     std::cout << "gait freq increase 0.5 " << std::endl;
//   }
//   else if (joy_msg.axes[7]  < 0 )
//   {
//     mobCmd_[4] -= 0.5;
//     std::cout << "gait freq decrease 0.5 " << std::endl;
//   }

}


void State_Rl::getCurrentObservation_Walk()
{
    Eigen::Vector4d q(_lowState->imu.quaternion[1],_lowState->imu.quaternion[2],_lowState->imu.quaternion[3],_lowState->imu.quaternion[0]);
    Eigen::Vector3d v(0.0,0.0,-1.0); 
    Eigen::Vector3d proj_gravity_eigen = quat_rotate_inverse(q, v);
    proj_gravity[0] = proj_gravity_eigen(0);
    proj_gravity[1] = proj_gravity_eigen(1);
    proj_gravity[2] = proj_gravity_eigen(2);

    // std::cout<<proj_gravity[0]<<" "<<proj_gravity[1]<<" "<<proj_gravity[2]<<std::endl;

    mobCmd_[0]=_lowState->userValue.lx;
    mobCmd_[1]=_lowState->userValue.ly;
    mobCmd_[2]=_lowState->userValue.rx;

    for (int i = 0; i < 3; i++)
    {
        obs_buf_[i] = proj_gravity[i];
    }

    for(int i=0;i<CMD_DIM;i++)
    {
        obs_buf_[i+3]=mobCmd_[i];
    }

    obs_buf_[3]*=2;
    obs_buf_[4]*=2;
    obs_buf_[5]*=0.25;
    obs_buf_[6]*=2;
    obs_buf_[12]*=0.15;
    obs_buf_[13]*=0.3;
    obs_buf_[14]*=0.3;


    for (int i = 0; i < NUM_DOFS; ++i)
    {
    //关节位置
        obs_buf_[i + 3 + CMD_DIM] = (_lowState->motorState[i].q - default_dof_pos[i]);
        //关节速度
        obs_buf_[i + 3 + NUM_DOFS + CMD_DIM] = _lowState->motorState[i].dq * 0.05; 
        //last action
        obs_buf_[i + 3 + NUM_DOFS * 2 + CMD_DIM] = action_cmd_Walk[i];
        //last_last_action
        obs_buf_[i + 3 + NUM_DOFS * 3 + CMD_DIM] = last_action_cmd_Walk[i];
    }

    float frequencies=mobCmd_[4],phases=mobCmd_[5],offsets=mobCmd_[6],bounds=mobCmd_[7];
    gait_indices=std::fmod(gait_indices + 0.02 * frequencies, 1.0);
    float foot_indices[4];
    foot_indices[0]=gait_indices+phases+offsets+bounds;
    foot_indices[1]=gait_indices+offsets;
    foot_indices[2]=gait_indices+bounds;
    foot_indices[3]=gait_indices+phases;

    obs_buf_[OBS_DIM-4]=std::sin(2*M_PI*foot_indices[0]);
    obs_buf_[OBS_DIM-3]=std::sin(2*M_PI*foot_indices[1]);
    obs_buf_[OBS_DIM-2]=std::sin(2*M_PI*foot_indices[2]);
    obs_buf_[OBS_DIM-1]=std::sin(2*M_PI*foot_indices[3]);

    float clip_obs=100.0;
    for(int i=0;i<OBS_DIM;i++)
    {
    if(obs_buf_[i]>clip_obs)obs_buf_[i]=100;
    if(obs_buf_[i]<-clip_obs)obs_buf_[i]=-100;
    }

    memmove(obs_history_Walk, obs_history_Walk + OBS_DIM, (NUM_OBS_IN_OBS_HISTORY - 1)*OBS_DIM * sizeof(float));
    memcpy(obs_history_Walk+(NUM_OBS_IN_OBS_HISTORY - 1)*OBS_DIM , obs_buf_ , OBS_DIM * sizeof(float));
}

Eigen::Vector3d State_Rl::quat_rotate_inverse(const Eigen::Vector4d& q, const Eigen::Vector3d& v) {
    double q_w = q[3];  // 提取四元数的实部 w
    Eigen::Vector3d q_vec = q.head<3>();  // 提取四元数的虚部 (x, y, z)

    Eigen::Vector3d a = v * (2.0 * q_w * q_w - 1.0);

    Eigen::Vector3d b = 2.0 * q_w * q_vec.cross(v);

    Eigen::Vector3d c = 2.0 * q_vec * (q_vec.dot(v));

    return a - b + c;
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