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
            gear_ratio=1;
        }
        else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
            
            // _lowCmd->setZeroGain(i);

            _lowCmd->setRealrlGain(i);
            gear_ratio=9.1;
        }
    }
    if(walk_these_ways){
        memset(action_cmd_Walk,0,sizeof(float)*NUM_DOFS);
        memset(last_action_cmd_Walk,0,sizeof(float)*NUM_DOFS);
        memset(action_history,0,sizeof(float)*ACTION_HISTORY_NUMS*NUM_DOFS);
        memset(obs_buf_,0,sizeof(float)*OBS_DIM);
        memset(obs_history_with_adaptation,0,sizeof(float)*(NUM_OBS_IN_OBS_HISTORY*OBS_DIM+2));
        memset(obs_history,0,sizeof(float)*NUM_OBS_IN_OBS_HISTORY*OBS_DIM);

        mobCmd_[0]=0;//x方向速度
        mobCmd_[1]=0;//y方向速度
        mobCmd_[2]=0;//yaw方向速度
        mobCmd_[3]=0.05;//身高
        mobCmd_[4]=3;//踏步频率
        mobCmd_[5]=0.5;//步态
        mobCmd_[6]=0.0;
        mobCmd_[7]=0.0;
        mobCmd_[8]=0.5;
        mobCmd_[9]=0.2;//步幅
        mobCmd_[10]=0.0;//pitch_cmd
        mobCmd_[11]=0.0;//roll_cmd
        mobCmd_[12]=0.25;//站姿宽度cmd
        mobCmd_[13]=0;//pitch_cmd
        mobCmd_[14]=0;//roll_cmd

        modle=0;
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
    time_rl=0;
    sin_counter = 0.0;
    output_angle_c = (_lowState->motorState[2].q ) * (180/PI);
    shape_control = false;
    modle_control = false;
    memset(obs_legged,0,sizeof(float)*N_proprio_legged);
    memset(encoder_input_legged,0,sizeof(float)*Num_observations_legged);
    memset(encoder_output_legged,0,sizeof(float)*Num_encoder_legged);
    memset(obs_history_legged,0,sizeof(float)*(History_len_legged*N_proprio_legged));
    memset(policy_input_legged,0,sizeof(float)*(Num_observations_legged+Num_encoder_legged));
    memset(action_cmd_legged,0,sizeof(float)*Num_dof);
    memset(last_action_cmd_legged,0,sizeof(float)*Num_dof);
    time_rl=0;
}

void State_Rl::run(){
    speed_limit();
    // test_motor();o
    time_rl++;
    if (0)
    {
        stateMachine_Walk();
        mnnInference_Walk();
        getCurrentObservation_Walk();
    }
    // else if(0){
    //     stateMachine_mast();
    //     mnnInference_mast();
    // }
    // else if(0){
    //     stateMachine_mujoco();
    //     mnnInference_mujoco();
    // }
    else if(0){
    //     // if (time_rl>30)
    //     // {
    //         // time_rl=0;
            stateMachine_Loco();
            mnnInference_Loco();
        }
    //     // usleep(200);
    //     // usleep(100);
    // }
    else if(1){
        if(time_rl>1){
            time_rl=0;
            stateMachine_legged();
            mnnInference_legged();
            std::cout<<"执行";
        }
        else{
            std::cout<<"没有执行";
        }
    }
    //     //     time_rl++;
    //     //     usleep(100);
    //     // }
    //     // else{
        // stateMachine_legged();
        // mnnInference_legged();
    //     // }
    //     // if (time_rl>100)
    //     // {
    //     //     time_rl=0;
    //     // }
    //     // usleep(100);
    // }
    // else if(0){
    //     backflip_time+=1;
    //     for (size_t i = 0; i < 4; i++)
    //     {
    //         _lowCmd->setSimbackfileGain(i);
    //     }
    //     if (backflip_time%50==0)
    //     {
    //         stateMachine_backflip();
    //         mnnInference_backflip();
    //     }
    //     if (backflip_time>1000)
    //     {
    //        time_zaro();
    //     }
        
    // }
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
void State_Rl::mnnInference_mast()
{
    Vec3 eu_ang;
    eu_ang = quaternion_to_euler_array(_lowState->imu.getQuat());
    std::cout<<"olx:"<<eu_ang[0]<<"\ty:"<<eu_ang[1]<<"\tz:"<<eu_ang[2]<<std::endl;
    for (size_t i = 0; i < 3; i++)
    {
        obs[0][i] = 0 *obs_scales_ang_vel;
        obs[0][i+3] = 0 *obs_scales_quat;
    }
    obs[0][6] = -_lowState->userValue.lx * obs_scales_lin_vel;
    obs[0][7] = -_lowState->userValue.ly * obs_scales_lin_vel;
    obs[0][8] = _lowState->userValue.rx ;
    for (size_t i = 0; i < 12; i++)
    {
        obs[0][9+i] = (_lowState->motorState[i].q-default_dof_pos[i]) *obs_scales_dof_pos;
        obs[0][21+i] = _lowState->motorState[i].dq * obs_scales_dof_vel;
        obs[0][33+i] = last_lowCmd[i];
    }
    for (size_t i = 0; i < N_proprio_mast; i++)
    {
        policy_input[0][i] = obs[0][i];
    }
    for (size_t i = 0; i < N_priv_latent_mast  + N_scan_Loco; i++)
    {
        policy_input[0][i+N_proprio_mast]=0;
    }
    for (size_t i = 0; i < History_len_mast*N_proprio_mast; i++)
    {
        policy_input[0][i+N_proprio_mast+N_priv_latent_mast+N_scan_mast]=obs_history[0][i];
    }
    for (size_t i = 0; i < (History_len_mast-1)*N_proprio_mast; i++)
    {
        obs_history[0][i] = obs_history[0][i+N_proprio_mast];
    }
    for (size_t i = 0; i < N_proprio_mast; i++)
    {
        obs_history[0][((History_len_mast-1)*N_proprio_mast)+i] = obs[0][i];
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

void State_Rl::stateMachine_mast(){
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
        _lowCmd->motorCmd[i].q = action_scaled[i]*0.5+ default_dof_pos[i];
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
      std::string mobModelPath = "../pd/body_latest.mnn";
      rlptr = std::make_shared<rl_Inference>(mobModelPath);
      rlptr->initBuffer();
    }
    rlptr->resetNode();
    if (adaptationNetPtr == nullptr)
    {
      std::string mobModelPath = "../pd/adaptation_module_latest.mnn";
      adaptationNetPtr = std::make_shared<rl_Inference>(mobModelPath);
      adaptationNetPtr->initBuffer();
    }
    mobRun();
    Pose_transformation();
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

    mobCmd_[0]=-_userValue.ly*2+0.3;
    mobCmd_[1]=-_userValue.lx*0;
    mobCmd_[2]=_userValue.rx*2;

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

void State_Rl::mnnInference_Loco()
{
    for (size_t i = 0; i < 3; i++)
    {
        action_stateq_Loco[i+3]=_lowState->motorState[i].q-default_dof_pos[i];
        action_statedq_Loco[i+3]=_lowState->motorState[i].dq;

        action_stateq_Loco[i]=_lowState->motorState[i+3].q-default_dof_pos[i+3];
        action_statedq_Loco[i]=_lowState->motorState[i+3].dq;

        action_stateq_Loco[i+9]=_lowState->motorState[i+6].q-default_dof_pos[i+6];
        action_statedq_Loco[i+9]=_lowState->motorState[i+6].dq;

        action_stateq_Loco[i+6]=_lowState->motorState[i+9].q-default_dof_pos[i+9];
        action_statedq_Loco[i+6]=_lowState->motorState[i+9].dq;
    }
    
    Eigen::Vector4d q(_lowState->imu.quaternion[1],_lowState->imu.quaternion[2],_lowState->imu.quaternion[3],_lowState->imu.quaternion[0]);
    Eigen::Vector3d v(0.0,0.0,-1.0); 
    Eigen::Vector3d proj_gravity_eigen = quat_rotate_inverse(q, v);
    Eigen::Vector3d line_v(_lowState->imu.line[0],_lowState->imu.line[1],_lowState->imu.line[2]);
    Eigen::Vector3d ang_v(_lowState->imu.gyroscope[0],_lowState->imu.gyroscope[1],_lowState->imu.gyroscope[2]);
    Eigen::Vector3d base_line= quat_rotate_inverse(q, line_v);
    Eigen::Vector3d base_ang= quat_rotate_inverse(q, ang_v);
    proj_gravity[0] = proj_gravity_eigen(0);
    proj_gravity[1] = proj_gravity_eigen(1);
    proj_gravity[2] = proj_gravity_eigen(2);
    // proj_gravity[0] = 0.0060272;
    // proj_gravity[1] = 0.018699;
    // proj_gravity[2] = -0.999807;
    std::cout<<"proj"<<proj_gravity[0]<<" "<<proj_gravity[1]<<" "<<proj_gravity[2]<<std::endl;
    for (size_t i = 0; i < 3; i++)
    {
        // obs_Loco[i] = base_line[i]*obs_scales_lin_vel;
        // obs_Loco[i+3] =_lowState->imu.gyroscope[i] *obs_scales_ang_vel;
        // obs_Loco[i+6] =proj_gravity[i];
        obs_Loco[i] = 0;
        obs_Loco[i+3] =0;
        obs_Loco[i+6] =proj_gravity[i];
    }
    // obs_Loco[9] = 2;
    // obs_Loco[10] = 0;
    // obs_Loco[11] = -5.2446e-04;
    obs_Loco[9] = -_userValue.lx * 0.8;
    obs_Loco[10] = 0;
    obs_Loco[11] = _userValue.rx *0.5;
    for (size_t i = 0; i < 12; i++)
    {
        obs_Loco[12+i] = action_stateq_Loco[i] *obs_scales_dof_pos;
        obs_Loco[24+i] = action_statedq_Loco[i]*obs_scales_dof_vel;
        // obs_Loco[24+i] = 0;
        obs_Loco[36+i] = last_action_cmd_Loco[i];
    }
    for (size_t i = 0; i < N_proprio_Loco; i++)
    {
        policy_input_Loco[i] =obs_Loco[i];
    }
    for (size_t i = 0; i < N_scan_Loco; i++)
    {
        policy_input_Loco[i+N_proprio_Loco]=0;
    }
    for (size_t i = 0; i <N_priv_latent_Loco; i++)
    {
        policy_input_Loco[i+N_proprio_Loco+N_priv_latent_Loco]=priv_latent[i];
    }
    for (size_t i = 0; i < History_len_Loco*N_proprio_Loco; i++)
    {
        policy_input_Loco[i+N_proprio_Loco+N_priv_latent_Loco+N_scan_Loco]=obs_history_Loco[i];
    }
    for (size_t i = 0; i < (History_len_Loco-1)*N_proprio_Loco; i++)
    {
        obs_history_Loco[i] = obs_history_Loco[i+N_proprio_Loco];
    }
    for (size_t i = 0; i < N_proprio_Loco; i++)
    {
        obs_history_Loco[((History_len_Loco-1)*N_proprio_Loco)+i] = obs_Loco[i];
    }
    // for (size_t i = (History_len_Loco-1)*N_proprio_Loco; i > 0; i--)
    // {
    //     obs_history_Loco[i+N_proprio_Loco] = obs_history_Loco[i];
    // }
    // for (size_t i = 0; i < N_proprio_Loco; i++)
    // {
    //     obs_history_Loco[i] = obs_Loco[i];
    // }
    // memmove(obs_history_Loco, obs_history_Loco + N_proprio_Loco, (History_len_Loco - 1)*N_proprio_Loco * sizeof(float));
    // memcpy(obs_history_Loco+(History_len_Loco - 1)*N_proprio_Loco , obs_Loco , N_proprio_Loco * sizeof(float));
    // for (size_t i = 48; i < 762; i++)
    // {
    //     policy_input_Loco[i]=0;
    // }
    // std::cout<<"policy_input";
    // for (size_t i = 0; i < 762; i++)
    // {
    //     std::cout<<"\t"<<policy_input_Loco[i];
    // }
    // std::cout<<std::endl;

    rlptr->advanceNNsync_Walk(policy_input_Loco,action_cmd_Loco);
    std::cout<<"action_cmd";
    for (size_t i = 0; i < 12; i++)
    {
        std::cout<<"\t"<<action_cmd_Loco[i];
    }       
    std::cout<<std::endl;
    float action_flt[12];
    for (size_t i = 0; i < 12; i++)
    {
        action_flt[i]=action_cmd_Loco[i]*0.8+last_action_cmd_Loco[i]*0.2;
    }
    for (size_t i = 0; i < 12; i++)
    {
        last_action_cmd_Loco[i]=action_cmd_Loco[i];
    }
    action_flt[0]*=0.5;
    action_flt[3]*=0.5;
    action_flt[6]*=0.5;
    action_flt[9]*=0.5;
    int soufang=0.25;
    for (size_t i = 0; i < 3; i++)
    {
        _lowCmd->motorCmd[i].q = action_flt[i+3] * 0.25 + default_dof_pos[i];
        _lowCmd->motorCmd[i+3].q = action_flt[i] * 0.25 + default_dof_pos[i+3];
        _lowCmd->motorCmd[i+6].q = action_flt[i+9] * 0.25 + default_dof_pos[i+6];
        _lowCmd->motorCmd[i+9].q = action_flt[i+6] * 0.25 + default_dof_pos[i+9];
    }
    // for (size_t i = 0; i < 12; i++)
    // {
    //     _lowCmd->motorCmd[i].q = action_flt[i] * 0.25 + default_dof_pos[i];
    // }
}

void State_Rl::stateMachine_Loco(){
    if (rlptr == nullptr)
    {
      std::string mobModelPath = "../LocomotionWithNP3O.mnn";
    // std::string mobModelPath = "../TOE.mnn";
      rlptr = std::make_shared<rl_Inference>(mobModelPath);
      rlptr->initBuffer();
    }
    rlptr->resetNode();
}


void State_Rl::stateMachine_legged(){
    if (rlptr == nullptr)
    {
      std::string mobModelPath = "../legged.mnn";
      rlptr = std::make_shared<rl_Inference>(mobModelPath);
      rlptr->initBuffer();
    }
    rlptr->resetNode();
    if (adaptationNetPtr == nullptr)
    {
      std::string mobModelPath = "../encoder_z_input.mnn";
      adaptationNetPtr = std::make_shared<rl_Inference>(mobModelPath);
      adaptationNetPtr->initBuffer();
    }


}

void State_Rl::mnnInference_legged()
{


    Eigen::Vector4d q(_lowState->imu.quaternion[1],_lowState->imu.quaternion[2],_lowState->imu.quaternion[3],_lowState->imu.quaternion[0]);
    Eigen::Vector3d v(0.0,0.0,-1.0); 
    Eigen::Vector3d line_v(_lowState->imu.line[0],_lowState->imu.line[1],_lowState->imu.line[2]);
    Eigen::Vector3d proj_gravity_eigen = quat_rotate_inverse(q, v);
    Eigen::Vector3d base_line= quat_rotate_inverse(q, line_v);
    proj_gravity[0] = proj_gravity_eigen(0);
    proj_gravity[1] = proj_gravity_eigen(1);
    proj_gravity[2] = proj_gravity_eigen(2);
    Eigen::Vector3d ang_v(_lowState->imu.gyroscope[0],_lowState->imu.gyroscope[1],_lowState->imu.gyroscope[2]);
    
    Eigen::Vector3d base_ang= quat_rotate_inverse(q, ang_v);
    
    for (size_t i = 0; i < 3; i++)
    {
        obs_legged[i] =_lowState->imu.gyroscope[i] *obs_scales_ang_vel;
    }
    
    for (size_t i = 0; i < 3; i++)
    {

        obs_legged[i+3] = proj_gravity[i];
    }
    obs_legged[6] = -_lowState->userValue.lx * obs_scales_lin_vel*0.8;
    obs_legged[7] = -_lowState->userValue.ly * obs_scales_lin_vel*0.8;
    obs_legged[8] = _lowState->userValue.rx *obs_scales_ang_vel*0.8;

    for (size_t i = 0; i < 12; i++)
    {
        obs_legged[9+i] = (_lowState->motorState[i].q-default_dof_pos[i]) *obs_scales_dof_pos;
        obs_legged[21+i] = _lowState->motorState[i].dq * obs_scales_dof_vel/9.1;
        obs_legged[33+i] = last_action_cmd_legged[i];
    }
    // std::cout << std::count_if(obs_legged, obs_legged + N_proprio_legged, [](float x) {return abs(x) < 0.000001; }) << std::endl;
    for (size_t i = 0; i < N_proprio_legged; i++)
    {
        encoder_input_legged[i] =obs_legged[i];
        // std::cout << obs_legged[i] << " ";
        
    }std::cout << std::endl;

    for (size_t i = 0; i < History_len_legged*N_proprio_legged; i++)
    {
        encoder_input_legged[i+N_proprio_legged]=obs_history_legged[i];
    }
    for (size_t i = 0; i <  (History_len_legged - 1)*N_proprio_legged; i++)
    {
        obs_history_legged[i]  = obs_history_legged[i+N_proprio_legged];
    }
    for (size_t i = 0; i <  N_proprio_legged; i++)
    {
        obs_history_legged[i+(History_len_legged - 1)*N_proprio_legged]  = obs_legged[i];
    }
    // memmove(obs_history_legged, obs_history_legged + N_proprio_legged, (History_len_legged - 1)*N_proprio_legged * sizeof(float));
    // memcpy(obs_history_legged+(History_len_legged - 1)*N_proprio_legged , obs_legged , N_proprio_legged * sizeof(float));
    adaptationNetPtr->advanceNNsync_Walk(encoder_input_legged,encoder_output_legged);
    
    for (size_t i = 0; i < Num_observations_legged; i++)
    {
        policy_input_legged[i] = encoder_input_legged[i];
    }

    for (size_t i = 0; i < Num_encoder_legged; i++)
    {
        policy_input_legged[i+Num_observations_legged] =encoder_output_legged[i];

    }
    // for (size_t i = 0; i < count; i++)
    // {
    //    policy_input_legged[i]=0;
    // }
    
    rlptr->advanceNNsync_Walk(policy_input_legged,action_cmd_legged);


    std::cout<<std::endl;
    float action_flt[12];
    for (size_t i = 0; i < 12; i++)
    {
        action_flt[i]=action_cmd_legged[i]*0.8+last_action_cmd_legged[i]*0.2;
        action_flt[i]*=0.25;
    }
    for (size_t i = 0; i < 12; i++)
    {
        last_action_cmd_legged[i]=action_cmd_legged[i];
    }
    action_flt[0]*=0.5;
    action_flt[3]*=0.5;
    action_flt[6]*=0.5;
    action_flt[9]*=0.5;

    for (size_t i = 0; i < 12; i++)
    {
        _lowCmd->motorCmd[i].q = action_flt[i]+ default_dof_pos[i];
    }
}


void State_Rl::mnnInference_mujoco()
{
    float pose_joint[12];
    float v_joint[12];
        for (size_t i = 0; i < 3; i++)
    {
        pose_joint[i+3]=_lowState->motorState[i].q-default_dof_pos[i];
        v_joint[i+3]=_lowState->motorState[i].dq;

        pose_joint[i]=_lowState->motorState[i+3].q-default_dof_pos[i+3];
        v_joint[i]=_lowState->motorState[i+3].dq;

        pose_joint[i+9]=_lowState->motorState[i+6].q-default_dof_pos[i+6];
        v_joint[i+9]=_lowState->motorState[i+6].dq;

        pose_joint[i+6]=_lowState->motorState[i+9].q-default_dof_pos[i+9];
        v_joint[i+6]=_lowState->motorState[i+9].dq;
    }
    Eigen::Vector4d q(_lowState->imu.quaternion[1],_lowState->imu.quaternion[2],_lowState->imu.quaternion[3],_lowState->imu.quaternion[0]);
    Eigen::Vector3d v(0.0,0.0,-1.0); 
    Eigen::Vector3d line_v(_lowState->imu.line[0],_lowState->imu.line[1],_lowState->imu.line[2]);
    Eigen::Vector3d proj_gravity_eigen = quat_rotate_inverse(q, v);
    Eigen::Vector3d base_line= quat_rotate_inverse(q, line_v);
    proj_gravity[0] = proj_gravity_eigen(0);
    proj_gravity[1] = proj_gravity_eigen(1);
    proj_gravity[2] = proj_gravity_eigen(2);

    std::cout<<"proj"<<proj_gravity[0]<<" "<<proj_gravity[1]<<" "<<proj_gravity[2]<<std::endl;
    for (size_t i = 0; i < 3; i++)
    {
        obs_legged[i] = base_line[i]*obs_scales_lin_vel;
        obs_legged[i+3] = _lowState->imu.gyroscope[i] *obs_scales_ang_vel;
        obs_legged[i+6] = proj_gravity[i];
    }
    for (size_t i = 0; i < 12; i++)
    {
        obs_legged[9+i] = pose_joint[i];
        obs_legged[21+i] = v_joint[i];
        obs_legged[33+i] = last_action_cmd_legged[i];
    }
    obs_legged[45] = -_lowState->userValue.lx * obs_scales_lin_vel;
    obs_legged[46] = -_lowState->userValue.ly * obs_scales_lin_vel;
    obs_legged[47] = _lowState->userValue.rx *obs_scales_ang_vel;
    rlptr->advanceNNsync_Walk(obs_legged,action_cmd_legged);
    std::cout<<std::endl;
    float action_flt[12];
    for (size_t i = 0; i < 12; i++)
    {
        action_flt[i]=action_cmd_legged[i];
    }
    for (size_t i = 0; i < 12; i++)
    {
        last_action_cmd_legged[i]=action_cmd_legged[i];
    }
    for (size_t i = 0; i < 3; i++)
    {
        _lowCmd->motorCmd[i].q = action_flt[i+3] * 0.5 + default_dof_pos[i+3];
        _lowCmd->motorCmd[i+3].q = action_flt[i] * 0.5 + default_dof_pos[i];
        _lowCmd->motorCmd[i+6].q = action_flt[i+9] * 0.5 + default_dof_pos[i+9];
        _lowCmd->motorCmd[i+9].q = action_flt[i+6] * 0.5 + default_dof_pos[i+6];
    }
    // for (size_t i = 0; i < 12; i++)
    // {
    //     _lowCmd->motorCmd[i].q = action_flt[i]*0.5+ default_dof_pos_mujoco[i];
    // }
}

void State_Rl::stateMachine_mujoco(){
    if (rlptr == nullptr)
    {
      std::string mobModelPath = "../mujoco.mnn";
      rlptr = std::make_shared<rl_Inference>(mobModelPath);
      rlptr->initBuffer();
    }
    rlptr->resetNode();
}

void State_Rl::mnnInference_backflip()
{
    float phase = 3.141592653589793f* backflip_time/5 * 0.02/2;
    Eigen::Vector4d q(_lowState->imu.quaternion[1],_lowState->imu.quaternion[2],_lowState->imu.quaternion[3],_lowState->imu.quaternion[0]);
    Eigen::Vector3d v(0.0,0.0,-1.0); 
    Eigen::Vector3d proj_gravity_eigen = quat_rotate_inverse(q, v);
    proj_gravity[0] = proj_gravity_eigen(0);
    proj_gravity[1] = proj_gravity_eigen(1);
    proj_gravity[2] = proj_gravity_eigen(2);
    for (size_t i = 0; i < 3; i++)
    {
        obs_backflip[i] = 0;
        obs_backflip[i+3] = proj_gravity[i];
    }
    for (size_t i = 0; i < 12; i++)
    {
        obs_backflip[6+i] = (_lowState->motorState[i].q-default_dof_pos[i]) *obs_scales_dof_pos;
        obs_backflip[18+i] = _lowState->motorState[i].dq * obs_scales_dof_vel;
        obs_backflip[30+i] = last_action_cmd_backflip[i];
        obs_backflip[42+i] = last_last_action_cmd_backflip[i];
    }
    obs_backflip[54] = sin(phase);
    obs_backflip[55] = cos(phase);
    obs_backflip[56] = sin(phase / 2);
    obs_backflip[57] = cos(phase / 2);
    obs_backflip[58] = sin(phase / 4);
    obs_backflip[59] = cos(phase / 4);
    // for (size_t i = 0; i < 60; i++)
    // {
    //     obs_backflip[i]=0;
    // }
    
    rlptr->advanceNNsync_Walk(obs_backflip,action_cmd_backflip);
    // std::cout<<std::endl;
    // float action_flt[12];
    // for (size_t i = 0; i < 12; i++)
    // {
    //     action_flt[i]=action_cmd_legged[i]*0.8+last_action_cmd_legged[i]*0.2;
    // }
    for (size_t i = 0; i < 12; i++)
    {
        last_last_action_cmd_backflip[i]=last_action_cmd_backflip[i];
    }
    for (size_t i = 0; i < 12; i++)
    {
        last_action_cmd_backflip[i]=action_cmd_backflip[i];
    }
    for (size_t i = 0; i < 12; i++)
    {
        _lowCmd->motorCmd[i].q = action_cmd_backflip[i]*0.5+ default_dof_pos[i];
    }
}

void State_Rl::stateMachine_backflip(){
    if (rlptr == nullptr)
    {
      std::string mobModelPath = "../backflip.mnn";
      rlptr = std::make_shared<rl_Inference>(mobModelPath);
      rlptr->initBuffer();
    }
    rlptr->resetNode();
}
void State_Rl::time_zaro(){
    backflip_time=0;
}
void State_Rl::mobRun()
{
  if ((int)_lowState->userValue.a == 1)  // trot
  {
    mobCmd_[5]=0.5;
    mobCmd_[6]=0;
    mobCmd_[7]=0;

  }
  else if ((int)_lowState->userValue.b == 1)  // pace
  {
    mobCmd_[5]=0;
    mobCmd_[6]=0;
    mobCmd_[7]=0.5;

  }
  else if ((int)_lowState->userValue.x == 1)  // pronk
  {
    mobCmd_[5]=0;
    mobCmd_[6]=0;
    mobCmd_[7]=0;

  }
  else if ((int)_lowState->userValue.y == 1)  // bound
  {
    mobCmd_[5]=0;
    mobCmd_[6]=0.5;
    mobCmd_[7]=0;
  }
}
void State_Rl::Pose_transformation(){
    if(modle>6){
        modle=0;
    }
    if (_lowState->userValue.yy>1&&modle_control==false)
    {
        modle_control=true;
        modle++;
    }
    else if (_lowState->userValue.yy<0&&modle_control==false)
    {
        modle_control=true;
        if (modle>0)
        {
            modle--;
        }
    }
    else if (_lowState->userValue.yy==0)
    {
        modle_control=false;
        // if (modle>0)
        // {
        //     modle--;
        // }
    }
    
    if ((int)modle == 0)  // theight
    {
        
        std::cout<<"\theight"<<mobCmd_[3]<<"\t";
        if (mobCmd_[3]+0.01<0.15&&_lowState->userValue.xx>1&&shape_control==false)
        {
            shape_control=true;
            mobCmd_[3]+=0.01;
        }
        else if (mobCmd_[3]-0.01>-0.25&&_lowState->userValue.xx<0&&shape_control==false)
        {
            shape_control=true;
            mobCmd_[3]-=0.01;
        }
        else if (_lowState->userValue.xx==0)
        {
            shape_control=false;
        }
        
    }
    else if ((int)modle == 1)  // step_frequency
    {
        std::cout<<"\tstep_frequency"<<mobCmd_[4]<<"\t";
        if (_lowState->userValue.xx>1&&shape_control==false)
        {
            shape_control=true;
            if (mobCmd_[4]<4)
            {
                mobCmd_[4]+=1;
            }
        }
        else if(_lowState->userValue.xx<0&&shape_control==false)
        {
            shape_control=true;
            if (mobCmd_[4]>2)
            {
                mobCmd_[4]-=1;
            }
        }
        else if (_lowState->userValue.xx==0)
        {
            shape_control=false;
        }
        

    }
    else if ((int)modle == 2)  //footswing_height
    {
        std::cout<<"\tfootswing_height"<<mobCmd_[9]<<"\t";
        if (_lowState->userValue.xx>1&&shape_control==false)
        {
            shape_control=true;
            if (mobCmd_[9]<0.35)
            {
                mobCmd_[9]+=0.01;
            }
        }
        else if(_lowState->userValue.xx<0&&shape_control==false)
        {
            shape_control=true;
            if (mobCmd_[9]>0.03)
            {
                mobCmd_[9]-=0.01;
            }
        }
        else if (_lowState->userValue.xx==0)
        {
            shape_control=false;
        }

    }
    else if ((int)modle == 3)  // pitch
    {
        std::cout<<"\tpitch"<<mobCmd_[10]<<"\t";
        if (_lowState->userValue.xx>1&&shape_control==false)
        {
            shape_control=true;
            if (mobCmd_[10]<0.4)
            {
                mobCmd_[10]+=0.05;
                mobCmd_[13]+=0.05;
            }
        }
        else if(_lowState->userValue.xx<0&&shape_control==false)
        {
            shape_control=true;
            if (mobCmd_[10]>-0.4)
            {
                mobCmd_[10]-=0.05;
                mobCmd_[13]-=0.05;
            }
        }
        else if (_lowState->userValue.xx==0)
        {
            shape_control=false;
        }
    }
    // else if ((int)modle == 4)  // roll
    // {
    //     std::cout<<"\troll"<<mobCmd_[11]<<"\t";
    //     if (_lowState->userValue.xx>1&&shape_control==false)
    //     {
    //         shape_control=true;
    //         if (mobCmd_[11]<1)
    //         {
    //             mobCmd_[11]+=0.05;
    //             mobCmd_[14]+=0.05;
    //         }
    //     }
    //     else if(_lowState->userValue.xx<0&&shape_control==false)
    //     {
    //         shape_control=true;
    //         if (mobCmd_[11]>-1)
    //         {
    //             mobCmd_[11]-=0.05;
    //             mobCmd_[14]-=0.05;
    //         }
    //     }
    //     else if (_lowState->userValue.xx==0)
    //     {
    //         shape_control=false;
    //     }

    // }
    else if ((int)modle == 4)  // 站姿宽度cmd
    {
        std::cout<<"\tstance_width"<<mobCmd_[12]<<"\t";
        if (_lowState->userValue.xx>1&&shape_control==false)
        {
            shape_control=true;
            if (mobCmd_[12]<0.45)
            {
                mobCmd_[12]+=0.01;
            }
        }
        else if(_lowState->userValue.xx<0&&shape_control==false)
        {
            shape_control=true;
            if (mobCmd_[12]>0.1)
            {
                mobCmd_[12]-=0.01;
            }
        }
        else if (_lowState->userValue.xx==0)
        {
            shape_control=false;
        }

    }
}


Eigen::Vector3d State_Rl::quat_rotate_inverse(const Eigen::Vector4d& q, const Eigen::Vector3d& v) {
    double q_w = q[3];  // 提取四元数的实部 w
    Eigen::Vector3d q_vec(q[0], q[1], q[2]);  // 提取四元数的虚部 xyz
    Eigen::Vector3d a = v * (2.0 * q_w * q_w - 1.0);

    // 计算b = cross(q_vec, v) * 2.0 * q_w
    Eigen::Vector3d b = q_vec.cross(v) * 2.0 * q_w;

    // 计算c = q_vec * (q_vec.transpose() * v) * 2.0
    Eigen::Vector3d c = q_vec * (q_vec.transpose() * v) * 2.0;
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
    pitch_y = - std::asin(t2);
    
    t3 = +2.0 * (w * z + x * y);
    t4 = +1.0 - 2.0 * (y * y + z * z);
    yaw_z =  std::atan2(t3, t4);
    return {roll_x, pitch_y, yaw_z};
}
void State_Rl::test_motor(){
    float output_angle_d;
    output_angle_d = output_angle_c + 10 * sin(2*PI*sin_counter);
    float rotor_angle_d = (output_angle_d * (PI/180));
    _lowCmd->motorCmd[2].q=rotor_angle_d;
}
void State_Rl::speed_limit(){

    if (std::abs(_lowState->userValue.lx - _userValue.lx) > 0.01) {
        // 根据差值的正负决定增加还是减少_userValue.lx
        _userValue.lx += (_lowState->userValue.lx > _userValue.lx) ? 0.01 : -0.01;
    } else {
        // 如果差值不大于0.1，直接设置为_lowState->userValue.lx的值
        _userValue.lx = _lowState->userValue.lx;
    }
    if (std::abs(_lowState->userValue.ly - _userValue.ly) > 0.01) {
        // 根据差值的正负决定增加还是减少_userValue.lx
        _userValue.ly += (_lowState->userValue.ly > _userValue.ly) ? 0.01 : -0.01;
    } else {
        // 如果差值不大于0.1，直接设置为_lowState->userValue.lx的值
        _userValue.ly = _lowState->userValue.ly;
    }
    if (std::abs(_lowState->userValue.rx - _userValue.rx) > 0.01) {
        // 根据差值的正负决定增加还是减少_userValue.lx
        _userValue.rx += (_lowState->userValue.rx > _userValue.rx) ? 0.01 : -0.01;
    } else {
        // 如果差值不大于0.1，直接设置为_lowState->userValue.lx的值
        _userValue.rx = _lowState->userValue.rx;
    }
    if (std::abs(_lowState->userValue.ry - _userValue.ry) > 0.01) {
        // 根据差值的正负决定增加还是减少_userValue.lx
        _userValue.ry += (_lowState->userValue.ry > _userValue.ry) ? 0.01 : -0.01;
    } else {
        // 如果差值不大于0.1，直接设置为_lowState->userValue.lx的值
        _userValue.ry = _lowState->userValue.ry;
    }
}